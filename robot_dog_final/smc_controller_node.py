import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import numpy as np
import csv 

try:
    from controller import Robot
except ImportError:
    Robot = None

# ===============KHỞI TẠO VÀ THÔNG SỐ HỆ THỐNG===============
class SMCControllerNode(Node):
    def __init__(self):
        super().__init__('smc_controller_node')
        self.get_logger().info('Đang khởi tạo kết nối với Webots và Real-time stream...')
        
        self.pub_q_actual = self.create_publisher(Float64, '/FL_knee/actual', 10)
        self.pub_q_desired = self.create_publisher(Float64, '/FL_knee/desired', 10)
        
        # Đổi header CSV để lưu tọa độ Cartesian (x, y, z) thay vì chỉ lưu góc khớp
        self.csv_file = open('smc_data_FL_cartesian.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x_des', 'y_des', 'z_des', 'x_act', 'y_act', 'z_act'])
        
        self.robot = None
        if Robot is not None:
            try:
                self.robot = Robot()
                step_val = self.robot.getBasicTimeStep()
                self.time_step = int(step_val) if step_val > 0 else 3
                self.get_logger().info('Kết nối Webots thành công!')
            except Exception as e:
                self.get_logger().error(f'Lỗi kết nối: {e}')
                self.time_step = 3
        else:
            self.time_step = 3
            
        self.l1, self.l2, self.l3 = 0.08, 0.20, 0.20
        self.h_body = 0.28 
        
        self.J_motor = 0.001 
        self.B_motor = 0.005 
        
        self.lam = 35.0      
        self.K = 15.0        
        self.Phi = 40.0      
        self.alpha = 0.6    
        
        self.m2, self.m3 = 0.12, 0.08
        self.g = 9.81
        
        self.M6 = np.array([
            [  1,   0,   0,   0,   0,   0,  0],
            [ -6,   6,   0,   0,   0,   0,  0],
            [ 15, -30,  15,   0,   0,   0,  0],
            [-20,  60, -60,  20,   0,   0,  0],
            [ 15, -60,  90, -60,  15,   0,  0],
            [ -6,  30, -60,  60, -30,   6,  0],
            [  1,  -6,  15, -20,  15,  -6,  1]
        ], dtype=float)
        
        self.legs = ["FL", "FR", "RL", "RR"]
        self.joint_names = ["hip_yaw", "hip_pitch", "knee_pitch"]
        self.gaits = {
            "trot":  {"beta": 0.5, "phases": [0.0, 0.5, 0.5, 0.0], "f": 1.5, "d_step": 0.3, "h_step": 0.08},
            "walk":  {"beta": 0.5, "phases": [0.0, 0.5, 0.25, 0.75], "f": 0.75, "d_step": 0.08, "h_step": 0.03},
             # Crawl: Đi chậm, bám đất nhiều hơn tí (beta=0.6) để không ngã, nhưng không để 0.88 gây kẹt chân.
            "crawl": {"beta": 0.6, "phases": [0.0, 0.5, 0.25, 0.75], "f": 0.5, "d_step": 0.1, "h_step": 0.04},
            
            # Pace: Hai chân cùng bên đi chung (lạc đà), rất dễ ngã ngang nên cần sải chân dài và dứt khoát.
            "pace":  {"beta": 0.5, "phases": [0.0, 0.5, 0.0, 0.5], "f": 1.0, "d_step": 0.2, "h_step": 0.05},
            
            # Bound: Chồm tới trước (như thỏ), cần lực đẩy (d_step) và độ nhấc cao (h_step) lớn để phóng đi.
            "bound": {"beta": 0.55, "phases": [0.0, 0.0, 0.5, 0.5], "f": 1.5, "d_step": 0.5, "h_step": 0.08},
            
            # Pronk: Nhảy tưng 4 chân. Đòi hỏi nén lò xo cực mạnh, nhấc chân rất cao để khỏi quẹt mỏ xuống sàn.
            "pronk": {"beta": 0.5, "phases": [0.0, 0.0, 0.0, 0.0], "f": 1.5, "d_step": 0.0, "h_step": 0.12},

            # Greet: Đứng 2 chân sau mừng chủ (Thông số bù nhìn để không lỗi key)
            "greet": {"beta": 1.0, "phases": [0.0, 0.0, 0.0, 0.0], "f": 0.0, "d_step": 0.0, "h_step": 0.0},
        }
        self.current_gait = "trot"
        self.q_prev = {leg: [0.0]*3 for leg in self.legs}
        self.qd_prev = {leg: [0.0]*3 for leg in self.legs}
        self.q_dot_filtered = {leg: [0.0]*3 for leg in self.legs}

        self.motors = {}
        self.sensors = {}
        if self.robot:
            for leg in self.legs:
                for joint in self.joint_names:
                    name = f"{leg}_{joint}"
                    self.motors[name] = self.robot.getDevice(name)
                    if self.motors[name]:
                        self.motors[name].setPosition(float('inf'))
                    self.sensors[name] = self.robot.getDevice(name + "_sensor")
                    if self.sensors[name]:
                        self.sensors[name].enable(self.time_step)

        self.pubs = {}
        for leg in self.legs:
            for joint in self.joint_names:
                name = f"{leg}_{joint}"
                self.pubs[name] = self.create_publisher(Float64, f'/{name}/cmd_effort', 10)

        self.first_run = True
        dt_seconds = self.time_step / 1000.0
        self.timer = self.create_timer(dt_seconds if dt_seconds > 0 else 0.003, self.control_loop)
# ==================================================

# ===============LUẬT ĐIỀU KHIỂN SMC===============
    def smc_law(self, q, q_dot, qd, qd_dot, joint_idx, leg):
        e = qd - q
        ed = qd_dot - q_dot
        s = ed + self.lam * e
        tau_eq = self.J_motor * (self.lam * ed) + self.B_motor * q_dot
        
        tau_g = 0.0
        if joint_idx == 1: 
            tau_g = (self.m2 * 0.1 + self.m3 * 0.2) * self.g * math.cos(q)
        elif joint_idx == 2:
            tau_g = self.m3 * 0.1 * self.g * math.cos(q)
            
        sat_s = np.clip(s / self.Phi, -1.0, 1.0)
        tau_sw = self.K * sat_s
        
        tau_new = tau_eq + tau_sw + tau_g
        return np.clip(tau_new, -30.0, 30.0)
# ==================================================

# ===============ĐỘNG HỌC NGƯỢC (IK)===============
    def inverse_kinematics(self, x, y, z, is_left):
        """ Tính góc khớp (q1, q2, q3) từ tọa độ (x, y, z) mong muốn """
        L_yz = math.sqrt(y**2 + z**2)
        if L_yz < self.l1: L_yz = self.l1
        z_p = -math.sqrt(L_yz**2 - self.l1**2)
        q1 = math.atan2(y, -z) - is_left * math.atan2(self.l1, -z_p)
        L_xz = math.sqrt(x**2 + z_p**2)
        cos_q3 = np.clip((L_xz**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3), -1.0, 1.0)
        q3 = -math.acos(cos_q3)
        alpha = math.atan2(-x, -z_p)
        beta = math.acos(np.clip((self.l2**2 + L_xz**2 - self.l3**2) / (2 * self.l2 * L_xz if L_xz != 0 else 0.001), -1.0, 1.0))
        return q1, alpha + beta, q3
# ==================================================

# ===============ĐỘNG HỌC THUẬN (FK)===============
    def forward_kinematics(self, q1, q2, q3, is_left):
        """ 
        Tính tọa độ (x, y, z) thực tế của bàn chân từ góc khớp (q1, q2, q3) đo được.
        Hàm này là ánh xạ ngược hoàn hảo của Inverse Kinematics ở trên.
        """
        # Tính tọa độ trong mặt phẳng X-Z phụ (Bỏ qua Hip Yaw l1)
        x = -(self.l2 * math.sin(q2) + self.l3 * math.sin(q2 + q3))
        z_p = -(self.l2 * math.cos(q2) + self.l3 * math.cos(q2 + q3))
        
        # Chiếu tọa độ về hệ quy chiếu gốc (Tính thêm Hip Yaw q1 và offset khâu l1)
        y = -z_p * math.sin(q1) + (is_left * self.l1) * math.cos(q1)
        z =  z_p * math.cos(q1) + (is_left * self.l1) * math.sin(q1)
        
        return x, y, z
# ==================================================

# ===============VÒNG LẶP ĐIỀU KHIỂN CHÍNH===============
    def control_loop(self):
        if not self.robot or self.robot.step(self.time_step) == -1:
            return

        t = self.robot.getTime()
        dt = self.time_step / 1000.0
        g = self.gaits[self.current_gait]
        
        for i, leg in enumerate(self.legs):
            is_left = 1 if (leg in ["FL", "RL"]) else -1
            
            # --- GIAI ĐOẠN 1: KHỞI ĐỘNG MỀM (3 giây đầu) ---
            if t < 3.0:
                ratio = t / 3.0
                current_z = -0.20 - (0.12 * ratio)
                pos = np.array([0.0, is_left * self.l1, current_z])
            
            # --- GIAI ĐOẠN 1.5: DÁNG MỪNG CHỦ (Đứng bằng 2 chân sau) ---
            elif self.current_gait == "greet":
                if leg in ["FL", "FR"]:
                    # Hai chân trước: Co rút lên sát ngực (z = -0.10) và rướn tới trước (x = 0.12)
                    # Dùng hàm sin() để làm 2 cái chân nhịp lên xuống (như đang lạy/vẫy tay)
                    wave = 0.03 * math.sin(8.0 * t) 
                    pos = np.array([0.12, is_left * self.l1, -0.10 + wave])
                else:
                    # Hai chân sau: Trụ vững. Nhích X tới trước một tí xíu để dồn trọng tâm về mông.
                    # Hạ Z thấp xuống xíu (-0.32) để gối chùng xuống giữ thăng bằng.
                    pos = np.array([0.03, is_left * self.l1, -0.32])

            # --- GIAI ĐOẠN 2: TẠO QUỸ ĐẠO BÉZIER (Cho các dáng đi bình thường) ---
            else:
                P_mid = np.array([0.0, is_left * self.l1, -0.32])
                
                t_p = ((t - 3.0) * g["f"] + g["phases"][i]) % 1.0
                beta_t = g["beta"]
                beta_u = 0.6  
                
                if t_p < (1.0 - beta_t) / 2.0:
                    u = t_p * (beta_u - 1.0) / (beta_t - 1.0)
                elif t_p <= (beta_t + 1.0) / 2.0:
                    u = (beta_u * (2.0 * t_p - 1.0)) / (2.0 * beta_t) + 0.5
                else:
                    u = beta_u / 2.0 - ((beta_u - 1.0) * (beta_t - 2.0 * t_p + 1.0)) / (2.0 * (beta_t - 1.0)) + 0.5
                
                d = g["d_step"]
                h = g["h_step"]
                
                W = np.array([
                    [0.0,            0.0, h],
                    [d/2.0 * 0.8,    0.0, h * 0.6],
                    [d/2.0,          0.0, h / 5.0],
                    [0.0,            0.0, 0.0],
                    [-d/2.0,         0.0, h / 5.0],
                    [-d/2.0 * 0.8,   0.0, h * 0.6],
                    [0.0,            0.0, h]
                ])
                
                U = np.array([1.0, u, u**2, u**3, u**4, u**5, u**6])
                pos_rel = U @ self.M6 @ W
                pos = P_mid + pos_rel
            
            # --- 1. TÍNH IK ĐỂ LẤY GÓC MONG MUỐN ---
            qd = self.inverse_kinematics(pos[0], pos[1], pos[2], is_left)
            
            # --- 2. ĐỌC CẢM BIẾN VÀ TÍNH FK ĐỂ LẤY TỌA ĐỘ THỰC TẾ ---
            q_actual_list = []
            for joint in self.joint_names:
                name = f"{leg}_{joint}"
                val = self.sensors[name].getValue() if (name in self.sensors and self.sensors[name]) else 0.0
                q_actual_list.append(val)
                
            x_act, y_act, z_act = self.forward_kinematics(q_actual_list[0], q_actual_list[1], q_actual_list[2], is_left)
            
            # Ghi dữ liệu quỹ đạo (Mong muốn vs Thực tế) của chân FL ra file CSV
            if leg == "FL":
                self.csv_writer.writerow([t, pos[0], pos[1], pos[2], x_act, y_act, z_act])
            
            # --- 3. ĐIỀU KHIỂN MOTOR BẰNG SMC ---
            for j, joint in enumerate(self.joint_names):
                name = f"{leg}_{joint}"
                q = q_actual_list[j]
                
                if self.first_run:
                    self.q_prev[leg][j], self.qd_prev[leg][j] = q, qd[j]
                
                q_dot_raw = (q - self.q_prev[leg][j]) / dt
                self.q_dot_filtered[leg][j] = self.alpha * self.q_dot_filtered[leg][j] + (1 - self.alpha) * q_dot_raw
                qd_dot = (qd[j] - self.qd_prev[leg][j]) / dt
                
                tau = self.smc_law(q, self.q_dot_filtered[leg][j], qd[j], qd_dot, j, leg)
                
                if name in self.motors and self.motors[name]:
                    self.motors[name].setTorque(float(tau))
                
                # Publish lên ROS Topic
                if leg == "FL" and joint == "knee_pitch":
                    msg_actual = Float64(); msg_actual.data = float(q)
                    self.pub_q_actual.publish(msg_actual)
                    msg_desired = Float64(); msg_desired.data = float(qd[j])
                    self.pub_q_desired.publish(msg_desired)
                
                self.q_prev[leg][j], self.qd_prev[leg][j] = q, qd[j]
                
        self.first_run = False
        
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()
# ==================================================

# ===============HÀM CHẠY CHÍNH (MAIN)===============
def main(args=None):
    rclpy.init(args=args)
    node = SMCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# ==================================================