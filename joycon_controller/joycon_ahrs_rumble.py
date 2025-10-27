# joycon_ahrs_rumble.py
import math, time
from typing import Tuple, Optional
from pyjoycon import JoyCon, get_R_id, get_L_id
import hid  # pip install hidapi

# ---- 物理換算係数（±2000 dps 前提：1 LSB = 0.07 deg/s）----
GYRO_LSB_TO_RAD = 0.07 * math.pi / 180.0  # ≈ 0.0012217304764 [rad/s] per LSB
REPORT_PERIOD = 0.015  # 1レポートにIMU3サンプル（~5ms/サンプル）

def _clamp(x, a, b): return max(a, min(b, x))

def _quat_from_dcm(i_b, j_b, k_b) -> Tuple[float, float, float, float]:
    # DCM -> Quaternion (w,x,y,z), right-handed, column vectors i_b,j_b,k_b
    m00, m01, m02 = i_b
    m10, m11, m12 = j_b
    m20, m21, m22 = k_b
    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S
    return (w, x, y, z)

def _euler_zyx_from_quat(q) -> Tuple[float, float, float]:
    # returns (yaw(Z), pitch(Y), roll(X)) in radians; aerospace ZYX
    w, x, y, z = q
    # yaw (Z)
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    # pitch (Y)
    sinp = 2.0*(w*y - z*x)
    sinp = _clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)
    # roll (X)
    sinr_cosp = 2.0*(w*x + y*z)
    cosr_cosp = 1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    return (yaw, pitch, roll)

class JoyConWithRumble(JoyCon):
    def __init__(self, *args, **kwargs):
        JoyCon.__init__(self,*args, **kwargs)
        
    def _send_rumble(self,data=b'\x00\x00\x00\x00\x00\x00\x00\x00'):
        self._RUMBLE_DATA = data
        self._write_output_report(b'\x10', b'', b'')

    def enable_vibration(self,enable=True):
        """Sends enable or disable command for vibration. Seems to do nothing."""
        self._write_output_report(b'\x01', b'\x48', b'\x01' if enable else b'\x00')
        
    def rumble_simple(self):
        """Rumble for approximately 1.5 seconds (why?). Repeat sending to keep rumbling."""
        print("揺れる")
        self._send_rumble(b'\x98\x1e\xc6\x47\x98\x1e\xc6\x47')

    def rumble_stop(self):
        """Instantly stops the rumble"""
        self._send_rumble()

class JoyConAHRSRumbler:
    """
    - DCM + complementary filter (gyro integral + accel gravity correction)
    - get_euler_rad(): (yaw, pitch, roll) [rad]
    - recenter_yaw(): zero the current yaw
    - rumble(): HD rumble (reverse-engineered encoding, JoyconLib 相当)
    """
    def __init__(self, side: str = "R", alpha: float = 0.02):
        """
        side: "R" or "L"
        alpha: accel信頼度（0..~0.2推奨）大きいほど重力に強く引き寄せ
        """
        assert side in ("R","L")
        self.side = side
        self.alpha = float(alpha)

        ids = get_R_id() if side == "R" else get_L_id()
        if None in ids:
            raise RuntimeError(f"Joy-Con {side} not found.")
        self.jc = JoyConWithRumble(*ids)

        # HID device for rumble (pyjoyconは0x10で固定ゼロ振幅を送るため、自前で送る)
        self.dev: Optional[hid.Device] = None
        try:
            self.dev = hid.Device(vendor_id=ids[0], product_id=ids[1], serial=ids[2])
        except Exception:
            self.dev = None  # 必要時のみ

        # DCM basis (body axes in world)
        self.i_b = [1.0, 0.0, 0.0]
        self.j_b = [0.0, 1.0, 0.0]
        self.k_b = [0.0, 0.0, 1.0]
        self.yaw_bias = 0.0

        self._pkt_ts = time.time()
        self._gn = [0.0, 0.0, 1.0]  # gravity (down) estimate
        self._global_count = 0

        # daemon callback
        self.jc.register_update_hook(self._on_report)

    # ------------- IMU fusion (DCM + complementary) -------------
    def _on_report(self, jc: JoyCon):
        # 1レポートに3サンプル
        # pyjoyconは連続readで最新reportだけ保持するので、近似的にdt=0.005 s×3で処理
        dt_sample = REPORT_PERIOD / 3.0  # ≈ 0.005s

        for s in (0,1,2):
            gx = jc.get_gyro_x(s) * GYRO_LSB_TO_RAD
            gy = jc.get_gyro_y(s) * GYRO_LSB_TO_RAD
            gz = jc.get_gyro_z(s) * GYRO_LSB_TO_RAD

            ax = float(jc.get_accel_x(s))
            ay = float(jc.get_accel_y(s))
            az = float(jc.get_accel_z(s))

            # 左右Joy-Conの座標補正（JoyconLib相当：LはY/Z反転）
            if self.side == "L":
                gy = -gy; gz = -gz
                ay = -ay; az = -az

            # 正規化重力
            norm = math.sqrt(ax*ax + ay*ay + az*az) or 1.0
            axn, ayn, azn = ax/norm, ay/norm, az/norm
            k_acc = [-axn, -ayn, -azn]  # down (−g方向)

            # ジャイロ微小角
            w_g = [-gx*dt_sample, -gy*dt_sample, -gz*dt_sample]

            # 誤差回転（k_b を k_acc へ回す微小回転）
            w_a = [
                self.k_b[1]*k_acc[2] - self.k_b[2]*k_acc[1],
                self.k_b[2]*k_acc[0] - self.k_b[0]*k_acc[2],
                self.k_b[0]*k_acc[1] - self.k_b[1]*k_acc[0],
            ]

            # 混合
            denom = 1.0 + self.alpha
            dth = [ (self.alpha*w_a[i] + w_g[i]) / denom for i in range(3) ]

            # DCM更新: v += dθ × v
            def _update(v, d):
                return [
                    v[0] + (d[1]*v[2] - d[2]*v[1]),
                    v[1] + (d[2]*v[0] - d[0]*v[2]),
                    v[2] + (d[0]*v[1] - d[1]*v[0]),
                ]
            self.i_b = _update(self.i_b, dth)
            self.j_b = _update(self.j_b, dth)
            self.k_b = _update(self.k_b, dth)

            # 再直交化（Gram–Schmidt）
            # 1) i,j の直交化
            err = 0.5*( self.i_b[0]*self.j_b[0] + self.i_b[1]*self.j_b[1] + self.i_b[2]*self.j_b[2] )
            i_tmp = [
                self.i_b[0] - err*self.j_b[0],
                self.i_b[1] - err*self.j_b[1],
                self.i_b[2] - err*self.j_b[2],
            ]
            j_tmp = [
                self.j_b[0] - err*self.i_b[0],
                self.j_b[1] - err*self.i_b[1],
                self.j_b[2] - err*self.i_b[2],
            ]
            # 2) 正規化
            def _norm(v):
                n = math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) or 1.0
                return [v[0]/n, v[1]/n, v[2]/n]
            self.i_b = _norm(i_tmp)
            self.j_b = _norm(j_tmp)
            # 3) k = i × j
            self.k_b = [
                self.i_b[1]*self.j_b[2] - self.i_b[2]*self.j_b[1],
                self.i_b[2]*self.j_b[0] - self.i_b[0]*self.j_b[2],
                self.i_b[0]*self.j_b[1] - self.i_b[1]*self.j_b[0],
            ]
            self.k_b = _norm(self.k_b)

    def get_quat(self) -> Tuple[float,float,float,float]:
        q = _quat_from_dcm(self.i_b, self.j_b, self.k_b)
        # yaw基準ずらし
        yaw, pitch, roll = _euler_zyx_from_quat(q)
        yaw -= self.yaw_bias
        # 再合成（Z軸回りだけ差し引き）
        cy, sy = math.cos(0.5*yaw), math.sin(0.5*yaw)
        cp, sp = math.cos(0.5*pitch), math.sin(0.5*pitch)
        cr, sr = math.cos(0.5*roll), math.sin(0.5*roll)
        # ZYX
        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy
        return (w,x,y,z)

    def get_euler_rad(self) -> Tuple[float,float,float]:
        q = self.get_quat()
        yaw,pitch,roll = _euler_zyx_from_quat(q)
        # 出力順を (pitch, roll, yaw) にしたい場合は並べ替え可
        return (pitch, roll, yaw)

    def recenter_yaw(self):
        # 現在のyawを0にリセット
        q = _quat_from_dcm(self.i_b, self.j_b, self.k_b)
        yaw,_,_ = _euler_zyx_from_quat(q)
        self.yaw_bias = yaw

    # ------------- HD Rumble (Output report 0x10) -------------
    def rumble(self, amplitude: float, side: Optional[str] = None,
               low_freq: float = 160.0, high_freq: float = 320.0, duration_ms: int = 120):
        """
        amplitude: 0..1（0で停止）
        side: None=このインスタンス側 / "L" or "R"
        """
        if self.dev is None:
            return
        side = side or self.side
        amp = _clamp(abs(amplitude), 0.0, 1.0)
        pkt = bytearray(49)
        pkt[0] = 0x10
        pkt[1] = self._global_count & 0x0F
        self._global_count = (self._global_count + 1) & 0x0F

        rumble = self._encode_rumble(low_freq, high_freq, amp)
        pkt[2:10] = rumble

        self.dev.write(pkt)
        if duration_ms > 0:
            time.sleep(duration_ms/1000.0)
            pkt[2:10] = b'\x00\x01\x40\x40\x00\x01\x40\x40'
            self.dev.write(pkt)
    
    def rumble_simple(self):
        self.jc.rumble_simple()

    def _encode_rumble(self, low_f: float, high_f: float, amplitude: float) -> bytes:
        # JoyconLib と dekuNukem の逆解析式に基づく近似エンコード
        if amplitude <= 0.0:
            return b'\x00\x01\x40\x40\x00\x01\x40\x40'

        lf = _clamp(low_f, 40.875885, 626.286133)
        hf = _clamp(high_f, 81.75177, 1252.572266)
        amp = _clamp(amplitude, 0.0, 1.0)

        hf_enc = int((round(32.0 * math.log(hf * 0.1, 2)) - 0x60) * 4) & 0xFFFF
        lf_enc = int(round(32.0 * math.log(lf * 0.1, 2)) - 0x40) & 0xFF

        if amp == 0:
            hf_amp = 0
        elif amp < 0.117:
            hf_amp = int(((math.log(amp * 1000.0, 2) * 32) - 0x60) / (5 - amp**2) - 1)
        elif amp < 0.23:
            hf_amp = int(((math.log(amp * 1000.0, 2) * 32) - 0x60) - 0x5C)
        else:
            hf_amp = int((((math.log(amp * 1000.0, 2) * 32) - 0x60) * 2) - 0xF6)
        hf_amp &= 0xFF

        lf_amp = int(round(hf_amp) * 0.5)
        parity = lf_amp % 2
        if parity > 0: lf_amp -= 1
        lf_amp = (lf_amp >> 1) + 0x40
        if parity > 0: lf_amp |= 0x8000

        buf = bytearray(8)
        buf[0] = hf_enc & 0xFF
        buf[1] = ((hf_enc >> 8) & 0xFF) + hf_amp
        buf[2] = (lf_enc + ((lf_amp >> 8) & 0xFF)) & 0xFF
        buf[3] = (lf_amp & 0xFF)
        buf[4:8] = buf[0:4]
        return bytes(buf)

# ---------------------- 簡易デモ ----------------------
if __name__ == "__main__":
    jc = JoyConAHRSRumbler("R", alpha=0.03)
    print("Press Ctrl+C to exit. Re-centering yaw in 1.0s...")
    time.sleep(1.0)
    jc.recenter_yaw()
    try:
        last_print = time.time()
        while True:
            if time.time() - last_print > 0.05:
                yaw, pitch, roll = jc.get_euler_rad()  # ZYX順を返す場合は関数内で調整
                # ここでは (pitch, roll, yaw) に並べ替えて表示
                p, r, y = pitch, roll, yaw
                print(f"pitch={p:+.3f} rad, roll={r:+.3f} rad, yaw={y:+.3f} rad", end="\r")
                last_print = time.time()
            time.sleep(0.001)
    except KeyboardInterrupt:
        jc.rumble(0.0)
        print("\nStopped.")
