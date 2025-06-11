import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from flask import Flask, Response, render_template_string
import threading
import cv2

# Flask setup
app = Flask(__name__)
car_data = {
    "speed": 0.0,
    "angular": 0.0,
    "stop_sign": False,
    "frame": None
}

# ROS2 节点：订阅速度、停止状态和相机
class WebStatusNode(Node):
    def __init__(self):
        super().__init__('web_status_node')
        self.bridge = CvBridge()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, '/stop_cmd_twist', self.stop_cmd_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def cmd_vel_callback(self, msg):
        car_data["speed"] = msg.linear.x
        car_data["angular"] = msg.angular.z

    def stop_cmd_callback(self, msg):
        is_stop = abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01
        car_data["stop_sign"] = is_stop

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            car_data["frame"] = frame
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

# Flask 视频流生成器 (快照式 MJPEG，每2秒刷新一次页面)
def generate_video():
    while True:
        frame = car_data["frame"]
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' +
                       jpeg.tobytes() + b'\r\n')

# HTML 模板：顶部一行三个状态框，下面摄像头画面
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>Jetson Car Dashboard</title>
  <!-- 保留每2秒刷新，展示“快照” -->
  <meta http-equiv="refresh" content="2">
  <style>
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background-color: #f9f9f9;
      color: #333;
      text-align: center;
      padding: 20px;
    }
    h1 {
      color: #007acc;
      font-size: 2em;
      margin-bottom: 20px;
    }
    /* 1. 顶部状态行：三个框水平排列 */
    .info-row {
      display: flex;
      justify-content: center;
      gap: 20px;
      margin-bottom: 20px;
    }
    .info-box {
      background: #fff;
      border-radius: 10px;
      padding: 15px 25px;
      box-shadow: 0 2px 5px rgba(0,0,0,0.1);
      font-size: 1.3em;
    }
    .stop-true  { color: red;   font-weight: bold; }
    .stop-false { color: green; font-weight: bold; }
    /* 2. 摄像头区域：独占一行 */
    .video-container {
      border: 5px solid #007acc;
      border-radius: 10px;
      display: inline-block;
      margin-top: 20px;
      padding: 10px;
    }
    .video-container h2 {
      margin: 0 0 10px;
      font-size: 1.4em;
    }
  </style>
</head>
<body>
  <h1>Jetson Car Dashboard</h1>
  <div class="info-row">
    <div class="info-box">Speed: {{ speed }}</div>
    <div class="info-box">Angular: {{ angular }}</div>
    <div class="info-box">
      Stop Sign Detected:
      <span class="{{ 'stop-true' if stop_sign else 'stop-false' }}">
        {{ stop_sign }}
      </span>
    </div>
  </div>
  <div class="video-container">
    <h2>Live Camera Feed</h2>
    <img src="/video_feed" width="640">
  </div>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE, **car_data)

@app.route("/video_feed")
def video_feed():
    return Response(
        generate_video(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

def main():
    rclpy.init()
    node = WebStatusNode()
    # 启动 ROS spinning 线程
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    # 启动 Flask 服务
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == "__main__":
    main()
