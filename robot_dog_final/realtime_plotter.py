import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading  # <--- Khai báo thêm thư viện xử lý đa luồng

class RealTimePlotter(Node):
    def __init__(self):
        super().__init__('realtime_plotter')
        
        # Lưu trữ dữ liệu để vẽ (tối đa 100 điểm ảnh cho mượt)
        self.max_points = 100
        self.time_data = []
        self.actual_data = []
        self.desired_data = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Biến đệm lưu góc đặt hiện tại để đồng bộ dữ liệu
        self.current_desired = 0.0

        # Subscribe lấy dữ liệu từ Node điều khiển
        self.create_subscription(Float64, '/FL_knee/actual', self.actual_callback, 10)
        self.create_subscription(Float64, '/FL_knee/desired', self.desired_callback, 10)

        # Cấu hình đồ thị Matplotlib
        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.line_actual, = self.ax.plot([], [], 'r-', label='Actual Angle (Góc thực)')
        self.line_desired, = self.ax.plot([], [], 'b--', label='Desired Angle (Góc đặt)')
        
        self.ax.set_title('Real-time SMC Tracking - FL Knee', fontweight='bold')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Angle (rad)')
        self.ax.legend()
        self.ax.grid(True)

    def desired_callback(self, msg):
        # Chỉ cập nhật biến đệm, chưa đưa vào mảng vẽ
        self.current_desired = msg.data

    def actual_callback(self, msg):
        # Lấy actual làm nhịp, khi có actual thì đẩy cả desired vào mảng
        # Cách này giúp 3 mảng time, actual, desired LUÔN CÙNG ĐỘ DÀI
        curr_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        self.time_data.append(curr_time)
        self.actual_data.append(msg.data)
        self.desired_data.append(self.current_desired) 
        
        self._truncate_data()

    def _truncate_data(self):
        # Cắt bớt phần tử cũ nếu vượt quá giới hạn 100 điểm
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.actual_data.pop(0)
            self.desired_data.pop(0)

    def update_plot(self, frame):
        # Cập nhật đồ thị (được gọi liên tục bởi FuncAnimation)
        if len(self.time_data) > 0:
            self.line_actual.set_data(self.time_data, self.actual_data)
            self.line_desired.set_data(self.time_data, self.desired_data)
            self.ax.relim()
            self.ax.autoscale_view()
        return self.line_actual, self.line_desired

def main():
    rclpy.init()
    node = RealTimePlotter()
    
    # Tách rclpy.spin() ra chạy ngầm ở một luồng riêng biệt
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # Dùng FuncAnimation để cập nhật đồ thị mỗi 50ms
    ani = FuncAnimation(node.fig, node.update_plot, interval=50, cache_frame_data=False)
    
    # Hàm plt.show() sẽ chặn luồng chính, nhưng luồng ROS vẫn chạy 
    plt.show()
    
    # Khi ông tắt cái cửa sổ đồ thị đi, code mới chạy xuống tới đây để dọn dẹp
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()