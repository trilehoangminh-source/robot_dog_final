import math
import numpy as np
from std_msgs.msg import Float64

class SMCPlugin:
    def init(self, webots_node, properties):
        # webots_node.robot chính là tương đương với Robot() cũ của ông
        self.robot = webots_node.robot
        self.node = webots_node
        
        self.time_step = int(self.robot.getBasicTimeStep())
        if self.time_step <= 0:
            self.time_step = 3
            
        # Publisher gửi sai số cho Nhật vẽ đồ thị
        self.error_pub = self.node.create_publisher(Float64, '/smc_error', 10)
        
        # --- THÔNG SỐ SMC ---
        self.l1, self.l2, self.l3 = 0.08, 0.20, 0.20
        self.J_motor, self.B_motor = 0.001, 0.005 
        self.lam, self.K, self.Phi, self.alpha = 30.0, 10.0, 5.0, 0.9     
        self.m2, self.m3 = 0.12, 0.08
        self.g = 9.81
        
        self.legs = ["FL", "FR", "RL", "RR"]
        self.joint_names = ["hip_yaw", "hip_pitch", "knee_pitch"]
        self.gaits = {
            "trot":  {"beta": 0.5, "phases": [0.0, 0.5, 0.5, 0.0], "f": 0.9, "d_step": 0.11, "h_step": 0.08}
        }
        self.current_gait = "trot" 
        
        self.q_prev = {leg: [0.0]*3 for leg in self.legs}
        self.qd_prev = {leg: [0.0]*3 for leg in self.legs}
        self.q_dot_filtered = {leg: [0.0]*3 for leg in self.legs}

        # Khởi tạo Motor & Sensor
        self.motors = {}
        self.sensors = {}
        for leg in self.legs:
            for joint in self.joint_names:
                name = f"{leg}_{joint}"
                self.motors[name] = self.robot.getDevice(name)
                if self.motors[name]:
                    self.motors[name].setPosition(float('inf'))
                self.sensors[name] = self.robot.getDevice(name + "_sensor")
                if self.sensors[name]:
                    self.sensors[name].enable(self.time_step)

    def smc_law(self, q, q_dot, qd, qd_dot, joint_idx):
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
        tau_new = tau_eq + (self.K * sat_s) + tau_g
        return np.clip(tau_new, -30.0, 30.0), e

    def inverse_kinematics(self, x, y, z, is_left):
        L_yz = math.sqrt(y**2 + z**2)
        L_yz = max(L_yz, self.l1)
        z_p = -math.sqrt(L_yz**2 - self.l1**2)
        q1 = math.atan2(y, -z) - is_left * math.atan2(self.l1, -z_p)
        L_xz = math.sqrt(x**2 + z_p**2)
        cos_q3 = np.clip((L_xz**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3), -1.0, 1.0)
        q3 = -math.acos(cos_q3)
        alpha = math.atan2(-x, -z_p)
        beta = math.acos(np.clip((self.l2**2 + L_xz**2 - self.l3**2) / (2 * self.l2 * L_xz if L_xz != 0 else 0.001), -1.0, 1.0))
        return q1, alpha + beta, q3

    def step(self):
        # Hàm này tự động được Webots gọi lặp đi lặp lại
        t = self.robot.getTime()
        dt = self.time_step / 1000.0
        if dt <= 0: return
        
        g = self.gaits[self.current_gait]
        total_error = 0.0
        
        for i, leg in enumerate(self.legs):
            is_left = 1 if (leg in ["FL", "RL"]) else -1
            
            if t < 3.0:
                ratio = t / 3.0
                current_z = -0.20 - (0.12 * ratio)
                pos = [0, is_left * self.l1, current_z]
            else:
                P_mid = [0, is_left * self.l1, -0.32] 
                t_local = ((t - 3.0) * g["f"] + g["phases"][i]) % 1.0
                if t_local < (1 - g["beta"]):
                    u = t_local / (1 - g["beta"])
                    P = [[-g["d_step"]/2, P_mid[1], P_mid[2]], [0, P_mid[1], P_mid[2]+g["h_step"]*1.5], [g["d_step"]/2, P_mid[1], P_mid[2]]]
                    pos = [(1-u)**2*P[0][0] + 2*(1-u)*u*P[1][0] + u**2*P[2][0], P_mid[1], (1-u)**2*P[0][2] + 2*(1-u)*u*P[1][2] + u**2*P[2][2]]
                else:
                    u = (t_local - (1 - g["beta"])) / g["beta"]
                    pos = [g["d_step"]/2 - u * g["d_step"], P_mid[1], P_mid[2]]
            
            qd = self.inverse_kinematics(pos[0], pos[1], pos[2], is_left)
            
            for j, joint in enumerate(self.joint_names):
                name = f"{leg}_{joint}"
                q = self.sensors[name].getValue() if self.sensors[name] else 0.0
                
                if t <= dt:
                    self.q_prev[leg][j], self.qd_prev[leg][j] = q, qd[j]
                
                q_dot_raw = (q - self.q_prev[leg][j]) / dt
                self.q_dot_filtered[leg][j] = self.alpha * self.q_dot_filtered[leg][j] + (1 - self.alpha) * q_dot_raw
                qd_dot = (qd[j] - self.qd_prev[leg][j]) / dt
                
                tau, error = self.smc_law(q, self.q_dot_filtered[leg][j], qd[j], qd_dot, j)
                total_error += abs(error)
                
                if self.motors[name]:
                    self.motors[name].setTorque(tau)
                
                self.q_prev[leg][j], self.qd_prev[leg][j] = q, qd[j]
                
        msg = Float64()
        msg.data = total_error
        self.error_pub.publish(msg)