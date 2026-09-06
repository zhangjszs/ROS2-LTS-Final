#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_visualizer.py
==================
订阅相机图像和视觉检测结果，在图像上绘制锥桶检测框，
通过 OpenCV 窗口实时显示，并可选保存为视频文件。

消息类型:
  图像:     sensor_msgs/Image          (via /resize_img_out 或 /camera/image_raw)
  检测结果: autodrive_msgs/HuatVisionDetections  (via /perception/vision/detections)

用法:
  # 直接运行（默认参数）
  python3 cone_visualizer.py

  # 以 ROS 节点参数方式启动（推荐）
  ros2 run vision_ros cone_visualizer --ros-args -p image_topic:=/resize_img_out -p save_video:=true

  # 搭配 vision_only_sim 使用
  ros2 launch fsd_launch vision_only_sim.launch.py bag:=/home/kerwin/rosbag/track.bag &
  ros2 run vision_ros cone_visualizer --ros-args -p image_topic:=/resize_img_out
"""

import os
import tempfile
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from autodrive_msgs.msg import HuatVisionDetections

# ──────────────────────────────────────────────────────────────────────────────
# 颜色类别定义
# color_types 枚举与 common_msgs/cone_types.h / HuatCone.type 保持一致
# ──────────────────────────────────────────────────────────────────────────────
COLOR_TYPE_BLUE = 0
COLOR_TYPE_YELLOW_SMALL = 1
COLOR_TYPE_YELLOW_BIG = 2
COLOR_TYPE_RED = 3
COLOR_TYPE_NONE = 4

# 类别显示名称 —— 与 HuatVisionDetections.msg 枚举保持一致
# 0=BLUE 1=YELLOW_SMALL 2=YELLOW_BIG 3=RED 4=NONE
CLASS_NAMES = {
    COLOR_TYPE_BLUE: "blue",
    COLOR_TYPE_YELLOW_SMALL: "yellow",
    COLOR_TYPE_YELLOW_BIG: "big_yellow",
    COLOR_TYPE_RED: "red",
    COLOR_TYPE_NONE: "cone?",  # 未被模型分类，仍显示框
}

# BGR 颜色（框颜色）
BOX_COLORS = {
    COLOR_TYPE_BLUE: (220, 60, 0),  # 蓝色
    COLOR_TYPE_YELLOW_SMALL: (0, 220, 240),  # 黄色
    COLOR_TYPE_YELLOW_BIG: (0, 160, 255),  # 橙色
    COLOR_TYPE_RED: (30, 30, 220),  # 红色
    COLOR_TYPE_NONE: (60, 60, 255),  # 洋红/未知 —— 醒目易区分
}

# 图像质量标签
IMAGE_QUALITY_LABELS = {0: "GOOD", 1: "DEGRADED", 2: "POOR", 3: "UNUSABLE"}
IMAGE_QUALITY_COLORS = {
    0: (60, 200, 60),
    1: (0, 200, 220),
    2: (0, 140, 255),
    3: (30, 30, 200),
}

# ──────────────────────────────────────────────────────────────────────────────
# 绘制参数
# ──────────────────────────────────────────────────────────────────────────────
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.55
FONT_THICK = 1
BOX_THICK = 2
CONF_SCALE = 1000.0  # HuatVisionDetections.confidences 是 0~1000 整数


# ──────────────────────────────────────────────────────────────────────────────
# 工具函数
# ──────────────────────────────────────────────────────────────────────────────
def draw_detection(img, cx, cy, bw, bh, color_type, confidence_raw):
    """在图像上绘制单个检测框、类别标签和置信度。

    Args:
        img: OpenCV BGR 图像（原地修改）
        cx, cy: bbox 中心坐标（像素）
        bw, bh: bbox 宽高（像素）
        color_type: int，COLOR_TYPE_* 枚举
        confidence_raw: int，0~1000
    """
    h, w = img.shape[:2]
    color = BOX_COLORS.get(color_type, BOX_COLORS[COLOR_TYPE_NONE])
    name = CLASS_NAMES.get(color_type, "unknown")
    conf = confidence_raw / CONF_SCALE

    # 计算框角点，做边界裁剪防止越界
    x1 = max(0, int(cx - bw * 0.5))
    y1 = max(0, int(cy - bh * 0.5))
    x2 = min(w - 1, int(cx + bw * 0.5))
    y2 = min(h - 1, int(cy + bh * 0.5))

    if x2 <= x1 or y2 <= y1:
        return  # 框退化，跳过

    # 检测框
    cv2.rectangle(img, (x1, y1), (x2, y2), color, BOX_THICK)

    # 标签文字：  blue_cone 0.92
    label = f"{name} {conf:.2f}"
    (tw, th), baseline = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICK)

    # 标签背景放在框上方；若空间不足则放框内
    label_y = y1 - 4
    if label_y - th < 0:
        label_y = y1 + th + 4

    bg_x1 = x1
    bg_y1 = label_y - th - baseline
    bg_x2 = x1 + tw + 2
    bg_y2 = label_y + baseline

    # 半透明背景
    sub = img[max(0, bg_y1) : max(0, bg_y2), max(0, bg_x1) : min(w, bg_x2)]
    if sub.size > 0:
        dark = np.zeros_like(sub)
        cv2.addWeighted(sub, 0.35, dark, 0.65, 0, sub)
        img[max(0, bg_y1) : max(0, bg_y2), max(0, bg_x1) : min(w, bg_x2)] = sub

    cv2.putText(img, label, (x1 + 1, label_y), FONT, FONT_SCALE, color, FONT_THICK, cv2.LINE_AA)

    # 中心点
    cv2.circle(img, (int(cx), int(cy)), 3, color, -1)


def draw_overlay(img, detections_msg, latency_ms):
    """在图像上绘制全部检测结果及 HUD 信息。

    Args:
        img: OpenCV BGR 图像（原地修改）
        detections_msg: HuatVisionDetections 消息，可为 None
        latency_ms: float，图像到可视化的端对端延迟

    Returns:
        n_drawn: int，实际绘制的检测框数
    """
    n_drawn = 0

    if detections_msg is not None and len(detections_msg.x) > 0:
        n = len(detections_msg.x)
        for i in range(n):
            ct = (
                detections_msg.color_types[i]
                if i < len(detections_msg.color_types)
                else COLOR_TYPE_NONE
            )
            cf = detections_msg.confidences[i] if i < len(detections_msg.confidences) else 0
            bw = detections_msg.bbox_widths[i] if i < len(detections_msg.bbox_widths) else 20.0
            bh = detections_msg.bbox_heights[i] if i < len(detections_msg.bbox_heights) else 40.0
            draw_detection(img, detections_msg.x[i], detections_msg.y[i], bw, bh, ct, cf)
            n_drawn += 1

    # ── HUD ──────────────────────────────────────────────────────────────────
    h, w = img.shape[:2]
    hud_lines = []

    if detections_msg is not None:
        qual_id = int(detections_msg.image_quality)
        qual_label = IMAGE_QUALITY_LABELS.get(qual_id, "?")
        qual_color = IMAGE_QUALITY_COLORS.get(qual_id, (180, 180, 180))
        infer_ms = detections_msg.inference_time_us / 1000.0
        backend = detections_msg.backend_name or "?"
        fallback = " [fallback]" if detections_msg.fallback_active else ""
        hud_lines.append(
            (f"Quality: {qual_label} ({detections_msg.quality_score:.2f})", qual_color)
        )
        hud_lines.append((f"Backend: {backend}{fallback}", (200, 200, 200)))
        hud_lines.append((f"Infer:   {infer_ms:.1f} ms", (200, 200, 200)))
        hud_lines.append((f"Detections: {n_drawn}", (200, 200, 200)))
    else:
        hud_lines.append(("Waiting for detections...", (80, 80, 200)))

    hud_lines.append((f"Latency: {latency_ms:.0f} ms", (160, 160, 160)))

    margin, line_h = 8, 20
    for k, (text, color) in enumerate(hud_lines):
        y = margin + (k + 1) * line_h
        cv2.putText(img, text, (margin, y), FONT, 0.48, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text, (margin, y), FONT, 0.48, color, 1, cv2.LINE_AA)

    return n_drawn


# ──────────────────────────────────────────────────────────────────────────────
# 主节点类
# ──────────────────────────────────────────────────────────────────────────────
class ConeVisualizer(Node):
    def __init__(self):
        super().__init__("cone_visualizer")

        # ── 参数 ──────────────────────────────────────────────────────────────
        self.image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        self.detection_topic = self.declare_parameter("detection_topic", "/perception/vision/detections").value
        self.window_name = self.declare_parameter("window_name", "Cone Visualizer").value
        self.display_scale = float(self.declare_parameter("display_scale", 1.0).value)
        self.save_video = bool(self.declare_parameter("save_video", False).value)
        default_video_path = os.path.join(tempfile.gettempdir(), "cone_viz.avi")
        self.video_path = self.declare_parameter("video_path", default_video_path).value
        self.video_fps = float(self.declare_parameter("video_fps", 10.0).value)
        self.max_det_age_sec = float(self.declare_parameter("max_det_age_sec", 5.0).value)
        # headless: true 时不调用 cv2.imshow / waitKey，仅写视频；适用于车载/CI/docker 等无显示环境
        # 若未显式设置且环境无 DISPLAY，自动进入 headless 模式
        explicit_headless = self.declare_parameter("headless", None).value
        if explicit_headless is None:
            self.headless = not bool(os.environ.get("DISPLAY"))
            if self.headless:
                self.get_logger().warn(
                    "[cone_visualizer] DISPLAY 未设置，自动进入 headless 模式（不开窗口）"
                )
        else:
            self.headless = bool(explicit_headless)

        # ── 内部状态 ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_image_ts = 0.0
        self.latest_dets = None
        self.latest_dets_ts = 0.0
        self.video_writer = None
        self.video_frame_wh = None

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(
            Image, self.image_topic, self._image_cb, 2
        )
        self.create_subscription(
            HuatVisionDetections, self.detection_topic, self._detection_cb, 5
        )

        self.get_logger().info("[cone_visualizer] image_topic:     %s" % self.image_topic)
        self.get_logger().info("[cone_visualizer] detection_topic: %s" % self.detection_topic)
        self.get_logger().info(
            "[cone_visualizer] save_video:      %s  ->  %s" % (self.save_video, self.video_path)
        )

    # ── 回调 ──────────────────────────────────────────────────────────────────
    def _image_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(
                "[cone_visualizer] cv_bridge error: %s" % e,
                throttle_duration_sec=5.0,
            )
            return
        # 始终使用墙钟接收时刻，避免 bag 回放时消息时间戳与 time.time() 相差数百万秒
        with self.lock:
            self.latest_image = bgr
            self.latest_image_ts = time.time()

    def _detection_cb(self, msg):
        # 始终使用墙钟接收时刻做新鲜度判断，与 bag 内嵌时间戳无关
        with self.lock:
            self.latest_dets = msg
            self.latest_dets_ts = time.time()

        # ── 一次性调试日志：打印实际收到的 color_type 值帮助排查 ──────────
        if not getattr(self, "_debug_logged", False) and len(msg.color_types) > 0:
            raw = [int(v) for v in msg.color_types]
            self.get_logger().info(
                "[cone_visualizer] FIRST detections: n=%d  color_types=%s  "
                "confidences=%s  backend=%s  fallback=%s"
                % (len(raw), raw[:10], list(msg.confidences)[:10], msg.backend_name, msg.fallback_active)
            )
            self._debug_logged = True

    # ── 视频写入 ──────────────────────────────────────────────────────────────
    def _init_video_writer(self, frame_wh):
        """初始化 VideoWriter，fourcc 优先 XVID，失败则 MJPG。"""
        if self.video_writer is not None:
            return
        os.makedirs(os.path.dirname(self.video_path) or ".", exist_ok=True)
        for fourcc_str in ("XVID", "MJPG"):
            fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
            writer = cv2.VideoWriter(self.video_path, fourcc, self.video_fps, frame_wh)
            if writer.isOpened():
                self.video_writer = writer
                self.video_frame_wh = frame_wh
                self.get_logger().info(
                    "[cone_visualizer] VideoWriter opened: %s  %s  fps=%.1f"
                    % (self.video_path, frame_wh, self.video_fps)
                )
                return
        self.get_logger().error("[cone_visualizer] Failed to open VideoWriter at %s" % self.video_path)

    def _write_frame(self, frame):
        """向视频文件写入一帧，自动做尺寸对齐。"""
        if not self.save_video:
            return
        wh = (frame.shape[1], frame.shape[0])
        if self.video_writer is None:
            self._init_video_writer(wh)
        if self.video_writer is None:
            return
        try:
            if wh != self.video_frame_wh:
                frame = cv2.resize(frame, self.video_frame_wh)
            self.video_writer.write(frame)
        except Exception as e:
            self.get_logger().error(
                "[cone_visualizer] Failed to write video frame: %s" % e,
                throttle_duration_sec=5.0,
            )

    # ── 主循环 ────────────────────────────────────────────────────────────────
    def _render_waiting_screen(self):
        placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Waiting for image...", (30, 180), FONT, 1.0, (100, 100, 200), 2, cv2.LINE_AA)
        cv2.putText(placeholder, f"topic: {self.image_topic}", (30, 220), FONT, 0.5, (140, 140, 140), 1, cv2.LINE_AA)
        cv2.imshow(self.window_name, placeholder)
        return cv2.waitKey(1) & 0xFF == ord("q")

    def _handle_keypress(self, img):
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("[cone_visualizer] 'q' pressed, shutting down")
            return True
        if key == ord("s") and not self.save_video:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".jpg", prefix="cone_viz_") as f:
                cv2.imwrite(f.name, img)
                self.get_logger().info("[cone_visualizer] Screenshot saved: %s" % f.name)
        return False

    def _render_and_show(self, img, dets, latency_ms):
        draw_overlay(img, dets, latency_ms)
        if not self.headless:
            if self.display_scale != 1.0:
                new_w = max(1, int(img.shape[1] * self.display_scale))
                new_h = max(1, int(img.shape[0] * self.display_scale))
                cv2.imshow(self.window_name, cv2.resize(img, (new_w, new_h)))
            else:
                cv2.imshow(self.window_name, img)
        self._write_frame(img)

    def spin(self):
        if not self.headless:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        rate = self.create_rate(30)

        while rclpy.ok():
            with self.lock:
                img = self.latest_image.copy() if self.latest_image is not None else None
                img_ts = self.latest_image_ts
                dets = self.latest_dets
                dets_ts = self.latest_dets_ts

            if img is None:
                if not self.headless and self._render_waiting_screen():
                    break
                rate.sleep()
                continue

            now = time.time()
            if dets is not None and now - dets_ts > self.max_det_age_sec:
                dets = None
            self._render_and_show(img, dets, (now - img_ts) * 1000.0)

            if not self.headless and self._handle_keypress(img):
                break
            rate.sleep()

        if not self.headless:
            cv2.destroyAllWindows()
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("[cone_visualizer] Video saved: %s" % self.video_path)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ConeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
