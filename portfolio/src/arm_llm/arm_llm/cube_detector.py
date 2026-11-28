#!/usr/bin/env python3
"""OpenCV 红/蓝方块识别节点。

订阅相机图像 → HSV 颜色分割 → 框出红/蓝方块 → 识别框上方标注颜色。
标注后的图像发布到 /cube_detection/annotated，供 Foxglove 的 Image 面板显示。

运行:
    python3 cube_detector.py
    换成深度相机的彩色图:
    python3 cube_detector.py --ros-args -p image_topic:=/camera_rgbd/image_raw
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


# ================= 配置区 =================
IMAGE_TOPIC = "/camera/image_raw"            # 订阅的相机话题（单目 RGB，在 base 上）
ANNOTATED_TOPIC = "/cube_detection/annotated"  # 标注后图像的发布话题
MIN_AREA = 500        # 轮廓最小面积，过滤小噪点
SHOW_WINDOW = False   # 是否弹本地窗口调试（无图形界面设 False；用 Foxglove 显示则 False）

# HSV 阈值 (OpenCV: H 0-180, S/V 0-255)。红色在色相环两端，需要两段。
RED_LOWER1 = np.array([0, 120, 80])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 80])
RED_UPPER2 = np.array([180, 255, 255])
BLUE_LOWER = np.array([100, 120, 80])
BLUE_UPPER = np.array([130, 255, 255])


def color_mask(hsv, color):
    """返回指定颜色的二值掩码。"""
    if color == "red":
        mask = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1) | \
               cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    else:
        mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    # 开运算去噪 + 闭运算补洞
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    return mask


def find_boxes(mask):
    """从掩码里提取外接矩形，过滤掉过小的轮廓。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in contours:
        if cv2.contourArea(c) < MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        boxes.append((x, y, w, h))
    return boxes


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector")
        self.declare_parameter("image_topic", IMAGE_TOPIC)
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub = self.create_publisher(Image, ANNOTATED_TOPIC, 10)
        self.get_logger().info(f"订阅 {image_topic}，发布标注图到 {ANNOTATED_TOPIC}")

    def image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge 转换失败: {e}")
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        annotated = img.copy()

        for color, label, bgr in (("red", "RED", (0, 0, 255)),
                                  ("blue", "BLUE", (255, 0, 0))):
            for (x, y, w, h) in find_boxes(color_mask(hsv, color)):
                cv2.rectangle(annotated, (x, y), (x + w, y + h), bgr, 2)
                # 标签放在框上方；快出画面顶部时改放到框内
                ty = y - 10 if y - 10 > 15 else y + 15
                cv2.putText(annotated, label, (x, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)

        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)

        if SHOW_WINDOW:
            cv2.imshow("cube_detection", annotated)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()