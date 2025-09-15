#!/usr/bin/env python3
import time, math, json, os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Header
from smbus import SMBus

G = 9.80665
ACC_SENS = 16384.0      # LSB/g @ ±2g
GYR_SENS_DPS = 131.0    # LSB/(deg/s) @ ±250 dps
DEG2RAD = math.pi / 180.0

# --- Registros MPU-6500 ---
REG_PWR_MGMT_1   = 0x6B
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CFG_2  = 0x1D
REG_WHO_AM_I     = 0x75
REG_ACCEL_XOUT_H = 0x3B

# --- Registros BMP280 ---
BMP_ID   = 0xD0
BMP_RST  = 0xE0
BMP_CTRL = 0xF4
BMP_CFG  = 0xF5
BMP_CAL0 = 0x88
BMP_DATA = 0xF7

def twos16(v): return v - 65536 if v & 0x8000 else v
def u16le(bus, addr, reg):
    lo = bus.read_byte_data(addr, reg); hi = bus.read_byte_data(addr, reg+1)
    return (hi << 8) | lo
def s16le(bus, addr, reg):
    v = u16le(bus, addr, reg)
    return v - 65536 if v & 0x8000 else v

class ImuBmpNode(Node):
    def __init__(self):
        super().__init__('imu_bmp_node')

        # --- Parámetros ---
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('imu_addr', 0x68)
        self.declare_parameter('bmp_addr', 0x76)
        self.declare_parameter('acc_cal_file', 'acc_cal.json')  # opcional (6-caras)
        self.declare_parameter('gyro_bias_secs', 2.0)           # auto-bias inicial en reposo

        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.imu_addr = int(self.get_parameter('imu_addr').value)
        self.bmp_addr = int(self.get_parameter('bmp_addr').value)
        self.acc_cal_file = str(self.get_parameter('acc_cal_file').value)
        self.gyro_bias_secs = float(self.get_parameter('gyro_bias_secs').value)

        # --- I2C ---
        self.bus = SMBus(1)
        self._init_mpu()
        self._init_bmp()

        # Calibraciones
        self.acc_cal = self._load_acc_cal(self.acc_cal_file)
        self.gyro_bias = self._quick_gyro_bias(self.gyro_bias_secs)

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 50)
        self.pub_p   = self.create_publisher(FluidPressure, 'baro/pressure', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(f'Publicando IMU+BMP a {self.rate_hz:.1f} Hz  frame_id={self.frame_id}')

    # ---------- INIT ----------
    def _init_mpu(self):
        a = self.imu_addr
        # reset y clock
        self.bus.write_byte_data(a, REG_PWR_MGMT_1, 0x80); time.sleep(0.1)
        self.bus.write_byte_data(a, REG_PWR_MGMT_1, 0x01); time.sleep(0.01)
        # rangos: ±250 dps, ±2g
        self.bus.write_byte_data(a, REG_GYRO_CONFIG,  0x00)
        self.bus.write_byte_data(a, REG_ACCEL_CONFIG, 0x00)
        # DLPF ≈ 20 Hz en gyro/acc
        self.bus.write_byte_data(a, REG_CONFIG,      0x04)
        self.bus.write_byte_data(a, REG_ACCEL_CFG_2, 0x04)
        # 1 kHz/(1+9) = 100 Hz
        self.bus.write_byte_data(a, REG_SMPLRT_DIV,  9)
        who = self.bus.read_byte_data(a, REG_WHO_AM_I)
        self.get_logger().info(f'MPU WHO_AM_I=0x{who:02X}')

    def _init_bmp(self):
        a = self.bmp_addr
        cid = self.bus.read_byte_data(a, BMP_ID)
        if cid not in (0x58, 0x60):  # 0x58 BMP280, 0x60 BME280
            self.get_logger().warn(f'BMP chip id 0x{cid:02X} inesperado')
        self.bus.write_byte_data(a, BMP_RST, 0xB6); time.sleep(0.05)
        # osrs_t=1, osrs_p=1, modo normal
        self.bus.write_byte_data(a, BMP_CTRL, 0x27)
        # filtro IIR x4, standby 125ms
        self.bus.write_byte_data(a, BMP_CFG,  0x14)
        # Calibración
        self.dig_T1 = u16le(self.bus, a, 0x88)
        self.dig_T2 = s16le(self.bus, a, 0x8A)
        self.dig_T3 = s16le(self.bus, a, 0x8C)
        self.dig_P1 = u16le(self.bus, a, 0x8E)
        self.dig_P2 = s16le(self.bus, a, 0x90)
        self.dig_P3 = s16le(self.bus, a, 0x92)
        self.dig_P4 = s16le(self.bus, a, 0x94)
        self.dig_P5 = s16le(self.bus, a, 0x96)
        self.dig_P6 = s16le(self.bus, a, 0x98)
        self.dig_P7 = s16le(self.bus, a, 0x9A)
        self.dig_P8 = s16le(self.bus, a, 0x9C)
        self.dig_P9 = s16le(self.bus, a, 0x9E)

    # ---------- DATA READ ----------
    def _read_acc_gyr(self):
        d = self.bus.read_i2c_block_data(self.imu_addr, REG_ACCEL_XOUT_H, 14)
        ax = twos16((d[0]<<8)|d[1]); ay = twos16((d[2]<<8)|d[3]); az = twos16((d[4]<<8)|d[5])
        gx = twos16((d[8]<<8)|d[9]);  gy = twos16((d[10]<<8)|d[11]); gz = twos16((d[12]<<8)|d[13])
        # unidades ROS: m/s^2 y rad/s
        ax, ay, az = [v/ACC_SENS*G for v in (ax,ay,az)]
        gx, gy, gz = [v/GYR_SENS_DPS*DEG2RAD for v in (gx,gy,gz)]
        return ax, ay, az, gx, gy, gz

    def _read_bmp280(self):
        a = self.bmp_addr
        d = self.bus.read_i2c_block_data(a, BMP_DATA, 6)
        adc_p = ((d[0]<<16)|(d[1]<<8)|d[2]) >> 4
        adc_t = ((d[3]<<16)|(d[4]<<8)|d[5]) >> 4
        # Temp (datasheet)
        var1 = (adc_t/16384.0 - self.dig_T1/1024.0) * self.dig_T2
        var2 = ((adc_t/131072.0 - self.dig_T1/8192.0)**2) * self.dig_T3
        t_fine = var1 + var2
        temp_c = t_fine / 5120.0
        # Presión (Pa)
        var1p = t_fine/2.0 - 64000.0
        var2p = var1p*var1p*self.dig_P6/32768.0
        var2p += var1p*self.dig_P5*2.0
        var2p = var2p/4.0 + self.dig_P4*65536.0
        var1p = (self.dig_P3*var1p*var1p/524288.0 + self.dig_P2*var1p)/524288.0
        var1p = (1.0 + var1p/32768.0) * self.dig_P1
        if var1p == 0: return temp_c, float('nan')
        p = 1048576.0 - adc_p
        p = (p - var2p/4096.0) * 6250.0 / var1p
        var1p = self.dig_P9 * p * p / 2147483648.0
        var2p = p * self.dig_P8 / 32768.0
        p = p + (var1p + var2p + self.dig_P7) / 16.0
        return temp_c, p  # Pascales

    # ---------- CALIBRACIÓN ----------
    def _load_acc_cal(self, path):
        if path and os.path.exists(path):
            with open(path) as f: return json.load(f)
        return {"offset":[0.0,0.0,0.0], "scale":[1.0,1.0,1.0]}

    def _apply_acc_cal(self, ax, ay, az):
        ox,oy,oz = self.acc_cal["offset"]; sx,sy,sz = self.acc_cal["scale"]
        return (ax-ox)*sx, (ay-oy)*sy, (az-oz)*sz

    def _quick_gyro_bias(self, secs):
        n = max(1, int(secs * self.rate_hz))
        sx=sy=sz=0.0
        for _ in range(n):
            _,_,_,gx,gy,gz = self._read_acc_gyr()
            sx+=gx; sy+=gy; sz+=gz
            time.sleep(1.0/self.rate_hz)
        return [sx/n, sy/n, sz/n]

    # ---------- LOOP ----------
    def _tick(self):
        ax,ay,az,gx,gy,gz = self._read_acc_gyr()
        ax,ay,az = self._apply_acc_cal(ax,ay,az)
        # compensa bias de gyro rápido (fijo)
        gx -= self.gyro_bias[0]; gy -= self.gyro_bias[1]; gz -= self.gyro_bias[2]
        temp_c, p_pa = self._read_bmp280()

        now = self.get_clock().now().to_msg()
        # IMU
        imu = Imu()
        imu.header = Header(stamp=now, frame_id=self.frame_id)
        imu.orientation.w = 1.0                     # sin estimación de orientación
        imu.orientation_covariance[0] = -1.0        # marca "desconocida"
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        # opcional: rellena covarianzas si conoces varianzas del sensor
        self.pub_imu.publish(imu)

        # Presión
        fp = FluidPressure()
        fp.header = Header(stamp=now, frame_id=self.frame_id)  # o 'baro_link' si usas otro frame
        fp.fluid_pressure = float(p_pa)  # Pascales
        # fp.variance = 0.0  # si la conoces
        self.pub_p.publish(fp)

def main():
    rclpy.init()
    node = ImuBmpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
