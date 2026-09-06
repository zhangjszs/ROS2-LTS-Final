#!/usr/bin/env python3

import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image

from autodrive_msgs.msg import HuatVisionDetections

try:
    import onnxruntime as ort
except ImportError:
    ort = None


# Cone color types — keep in sync with common_msgs/cone_types.h and HuatCone.type
# 视觉端输出统一的 YELLOW(1)，LiDAR 尺寸只用来把黄锥拆成 YELLOW_SMALL/YELLOW_BIG
BLUE = 0
YELLOW = 1  # 视觉端输出统一黄色
YELLOW_SMALL = 1  # 兼容别名
YELLOW_BIG = 2
RED = 3
NONE = 4

QUALITY_GOOD = 0
QUALITY_DEGRADED = 1
QUALITY_POOR = 2
QUALITY_UNUSABLE = 3

STATE_NORMAL = 0
STATE_DEGRADED = 1
STATE_FALLBACK = 2
STATE_VISION_LOST = 3

# (counter_attr, threshold_attr, next_state) — 按优先级顺序
_STATE_TRANSITIONS = {
    STATE_NORMAL: [
        ("consecutive_unusable", "unusable_frame_count", STATE_VISION_LOST),
        ("consecutive_poor", "poor_frame_count", STATE_FALLBACK),
        ("consecutive_degraded", "degraded_frame_count", STATE_DEGRADED),
    ],
    STATE_DEGRADED: [
        ("consecutive_unusable", "unusable_frame_count", STATE_VISION_LOST),
        ("consecutive_poor", "poor_frame_count", STATE_FALLBACK),
        ("consecutive_good", "recovery_frame_count", STATE_NORMAL),
    ],
    STATE_FALLBACK: [
        ("consecutive_unusable", "unusable_frame_count", STATE_VISION_LOST),
        ("consecutive_good", "recovery_frame_count", STATE_NORMAL),
        ("consecutive_degraded", "recovery_frame_count", STATE_DEGRADED),
    ],
    STATE_VISION_LOST: [
        ("consecutive_good", "recovery_frame_count", STATE_NORMAL),
        ("consecutive_degraded", "recovery_frame_count", STATE_DEGRADED),
        ("consecutive_poor", "recovery_frame_count", STATE_FALLBACK),
    ],
}


class Detection:
    __slots__ = ("x", "y", "w", "h", "confidence", "class_id", "color_type")

    def __init__(self, x, y, w, h, confidence, class_id, color_type):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.confidence = confidence
        self.class_id = class_id
        self.color_type = color_type


class PendingImageFrame:
    __slots__ = ("msg", "receive_mono")

    def __init__(self, msg, receive_mono):
        self.msg = msg
        self.receive_mono = float(receive_mono)


class LatestFrameBuffer:
    __slots__ = (
        "stale_after_sec",
        "_pending",
        "replaced_total",
        "stale_total",
        "last_stale_age_ms",
    )

    def __init__(self, stale_after_sec):
        self.stale_after_sec = float(stale_after_sec)
        self._pending = None
        self.replaced_total = 0
        self.stale_total = 0
        self.last_stale_age_ms = 0.0

    def store(self, frame):
        if self._pending is not None:
            self.replaced_total += 1
        self._pending = frame

    def has_pending(self):
        return self._pending is not None

    def pending_depth(self):
        return 1 if self._pending is not None else 0

    def take_latest(self, now_mono):
        frame = self._pending
        self._pending = None
        if frame is None:
            return None

        age_sec = float(now_mono) - frame.receive_mono
        if self.stale_after_sec > 0.0 and age_sec > self.stale_after_sec:
            self.stale_total += 1
            self.last_stale_age_ms = max(age_sec * 1000.0, 0.0)
            return None

        return frame


class FrameStageTiming:
    __slots__ = (
        "header_stamp_sec",
        "receive_mono",
        "pick_mono",
        "preprocess_done_mono",
        "inference_done_mono",
        "postprocess_done_mono",
        "publish_done_mono",
        "publish_ros_sec",
    )

    def __init__(self, header_stamp_sec, receive_mono):
        self.header_stamp_sec = float(header_stamp_sec)
        self.receive_mono = float(receive_mono)
        self.pick_mono = float(receive_mono)
        self.preprocess_done_mono = float(receive_mono)
        self.inference_done_mono = float(receive_mono)
        self.postprocess_done_mono = float(receive_mono)
        self.publish_done_mono = float(receive_mono)
        self.publish_ros_sec = 0.0

    def mark_picked(self, pick_mono):
        self.pick_mono = float(pick_mono)
        self.preprocess_done_mono = self.pick_mono
        self.inference_done_mono = self.pick_mono
        self.postprocess_done_mono = self.pick_mono
        self.publish_done_mono = self.pick_mono

    def mark_preprocess_done(self, done_mono):
        self.preprocess_done_mono = float(done_mono)

    def mark_inference_done(self, done_mono):
        self.inference_done_mono = float(done_mono)

    def mark_postprocess_done(self, done_mono):
        self.postprocess_done_mono = float(done_mono)

    def mark_publish_done(self, done_mono, publish_ros_sec):
        self.publish_done_mono = float(done_mono)
        self.publish_ros_sec = float(publish_ros_sec)

    @staticmethod
    def _delta_ms(start_mono, end_mono):
        return max((float(end_mono) - float(start_mono)) * 1000.0, 0.0)

    def snapshot(self):
        end_to_end_lag_ms = 0.0
        if self.publish_ros_sec > 0.0 and self.header_stamp_sec > 0.0:
            end_to_end_lag_ms = max((self.publish_ros_sec - self.header_stamp_sec) * 1000.0, 0.0)

        return {
            "receive_to_pick_ms": self._delta_ms(self.receive_mono, self.pick_mono),
            "preprocess_ms": self._delta_ms(self.pick_mono, self.preprocess_done_mono),
            "inference_stage_ms": self._delta_ms(
                self.preprocess_done_mono, self.inference_done_mono
            ),
            "postprocess_ms": self._delta_ms(self.inference_done_mono, self.postprocess_done_mono),
            "publish_ms": self._delta_ms(self.postprocess_done_mono, self.publish_done_mono),
            "total_processing_ms": self._delta_ms(self.pick_mono, self.publish_done_mono),
            "end_to_end_publish_lag_ms": end_to_end_lag_ms,
        }


class TemporalTracker:
    def __init__(self, iou_threshold, max_miss, min_hits):
        self.iou_threshold = float(iou_threshold)
        self.max_miss = int(max_miss)
        self.min_hits = int(min_hits)
        self.tracks = []

    @staticmethod
    def _iou(a, b):
        ax1, ay1 = a.x - a.w * 0.5, a.y - a.h * 0.5
        ax2, ay2 = a.x + a.w * 0.5, a.y + a.h * 0.5
        bx1, by1 = b.x - b.w * 0.5, b.y - b.h * 0.5
        bx2, by2 = b.x + b.w * 0.5, b.y + b.h * 0.5
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        union = a.w * a.h + b.w * b.h - inter
        return inter / union if union > 0.0 else 0.0

    def update(self, detections):
        for track in self.tracks:
            track["misses"] += 1

        matched = [False] * len(detections)
        for track in self.tracks:
            best_iou = 0.0
            best_idx = -1
            for idx, det in enumerate(detections):
                if matched[idx]:
                    continue
                iou = self._iou(track["det"], det)
                if iou > best_iou:
                    best_iou = iou
                    best_idx = idx
            if best_idx >= 0 and best_iou >= self.iou_threshold:
                track["det"] = detections[best_idx]
                track["hits"] += 1
                track["misses"] = 0
                matched[best_idx] = True

        for idx, det in enumerate(detections):
            if not matched[idx]:
                self.tracks.append({"det": det, "hits": 1, "misses": 0})

        self.tracks = [t for t in self.tracks if t["misses"] <= self.max_miss]
        return [t["det"] for t in self.tracks if t["hits"] >= self.min_hits]


class VisionNodePy(Node):
    _STATE_DIAG = {
        STATE_NORMAL: (DiagnosticStatus.OK, "Vision operating normally"),
        STATE_DEGRADED: (DiagnosticStatus.WARN, "Vision degraded - image quality reduced"),
        STATE_FALLBACK: (DiagnosticStatus.WARN, "Vision fallback - using HSV color detection"),
        STATE_VISION_LOST: (DiagnosticStatus.ERROR, "Vision lost - image unusable"),
    }
    _COLOR_BGR = {BLUE: (255, 0, 0), YELLOW: (0, 255, 255), YELLOW_BIG: (0, 100, 255), RED: (0, 0, 255)}

    def __init__(self):
        super().__init__("vision_node_py")
        self.bridge = CvBridge()
        self.frame_count = 0

        self.state = STATE_NORMAL
        self.consecutive_degraded = 0
        self.consecutive_poor = 0
        self.consecutive_unusable = 0
        self.consecutive_good = 0

        self.last_debug_pub = 0
        self.last_diag_pub = 0
        self.last_stage_timing = None
        self.skipped_inference_newer_pending = 0
        self.skipped_postprocess_newer_pending = 0
        self.last_fallback_ms = 0.0
        self.last_tracker_ms = 0.0
        self.last_image_received_mono = 0.0  # watchdog: 监控图像 topic 心跳

        self._load_params()
        self._init_backend()
        if not self.backend_ready and self.fallback_enabled:
            self.backend_name = "fallback_hsv"
        self.tracker = TemporalTracker(
            self.tracker_iou_threshold, self.tracker_max_miss, self.tracker_min_hits
        )

        self.detections_pub = self.create_publisher(
            HuatVisionDetections, "/perception/vision/detections", 1
        )
        self.debug_image_pub = self.create_publisher(
            Image, "/perception/vision/debug_image", 1
        )
        self.diag_pub_local = self.create_publisher(
            DiagnosticArray, "/perception/vision/diagnostics", 1
        )
        self.diag_pub_global = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1
        )

        self.latest_frame_buffer = LatestFrameBuffer(self.stale_frame_age_sec)
        self.latest_frame_cv = threading.Condition()
        self.worker_shutdown = False
        self.worker = threading.Thread(
            target=self._worker_loop, name="vision_node_py_worker", daemon=True
        )
        self.worker.start()
        self.context.on_shutdown(self._shutdown_worker)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, 1
        )

        # Watchdog timer: 即使图像断流也持续上报诊断，避免 /diagnostics 心跳消失
        self.image_stale_threshold_sec = float(
            self.declare_parameter("node/image_stale_threshold_sec", 1.0).value
        )
        self.watchdog_timer = self.create_timer(
            1.0 / max(self.diag_rate_hz, 0.1), self._watchdog_cb
        )

        self.get_logger().info(
            "[vision_node_py] Initialized - backend=%s, mode=%s, topic=%s, tracker=%s, "
            "fallback=%s, stale_frame_age_sec=%.3f, skip_heavy_if_newer_pending=%s, "
            "publish_debug_image=%s",
            self.backend_name,
            "onnx_inference" if self.backend_ready else "fallback_only",
            self.image_topic,
            "on" if self.tracker_enabled else "off",
            "on" if self.fallback_enabled else "off",
            self.stale_frame_age_sec,
            "on" if self.skip_heavy_if_newer_pending else "off",
            "on" if self.publish_debug_image else "off",
        )

    def _load_params(self):
        self.image_topic = self.declare_parameter("node/image_topic", "/camera/image_raw").value
        self.max_detections = int(self.declare_parameter("detection/max_detections", 50).value)
        self.require_model = bool(self.declare_parameter("node/require_model", True).value)

        self.backend_type = self.declare_parameter("inference/backend_type", "onnx").value
        self.model_path = self.declare_parameter("inference/model_path", "").value
        self.input_width = int(self.declare_parameter("inference/input_width", 1152).value)
        self.input_height = int(self.declare_parameter("inference/input_height", 1152).value)
        self.conf_threshold = float(self.declare_parameter("detection/conf_threshold", 0.55).value)
        self.nms_threshold = float(self.declare_parameter("detection/nms_threshold", 0.35).value)
        self.num_threads = int(self.declare_parameter("inference/num_threads", 2).value)

        self.publish_debug_image = bool(self.declare_parameter("output/publish_debug_image", False).value)
        self.debug_image_rate = float(self.declare_parameter("output/debug_image_rate", 5.0).value)
        self.diag_rate_hz = float(self.declare_parameter("node/diag_rate_hz", 2.0).value)
        self.stale_frame_age_sec = float(self.declare_parameter("node/stale_frame_age_sec", 0.5).value)
        self.skip_heavy_if_newer_pending = bool(
            self.declare_parameter("node/skip_heavy_if_newer_pending", True).value
        )

        self.confidence_scale = float(self.declare_parameter("output/confidence_scale", 1000.0).value)
        self.fallback_enabled = bool(self.declare_parameter("fallback/enabled", True).value)
        self.model_confidence_floor = float(
            self.declare_parameter("fallback/model_confidence_floor", 0.3).value
        )
        self.fallback_min_area = float(self.declare_parameter("fallback/min_contour_area", 200.0).value)
        self.fallback_max_area = float(self.declare_parameter("fallback/max_contour_area", 50000.0).value)

        self.tracker_enabled = bool(self.declare_parameter("tracker/enabled", True).value)
        self.tracker_iou_threshold = float(self.declare_parameter("tracker/iou_threshold", 0.4).value)
        self.tracker_max_miss = int(self.declare_parameter("tracker/max_miss", 1).value)
        self.tracker_min_hits = int(self.declare_parameter("tracker/min_hits", 2).value)

        self.degraded_frame_count = int(self.declare_parameter("node/degraded_frame_count", 3).value)
        self.poor_frame_count = int(self.declare_parameter("node/poor_frame_count", 5).value)
        self.unusable_frame_count = int(self.declare_parameter("node/unusable_frame_count", 10).value)
        self.recovery_frame_count = int(self.declare_parameter("node/recovery_frame_count", 5).value)

        self.quality_thresholds = {
            "blur_good": float(self.declare_parameter("quality/blur_threshold", 200.0).value),
            "blur_degraded": float(self.declare_parameter("quality/blur_degraded", 100.0).value),
            "blur_poor": float(self.declare_parameter("quality/blur_poor", 50.0).value),
            "brightness_low": float(self.declare_parameter("quality/brightness_low", 40.0).value),
            "brightness_high": float(self.declare_parameter("quality/brightness_high", 220.0).value),
            "brightness_very_low": float(self.declare_parameter("quality/brightness_very_low", 15.0).value),
            "brightness_very_high": float(self.declare_parameter("quality/brightness_very_high", 250.0).value),
            "overexposure_limit": float(self.declare_parameter("quality/overexposure_limit", 0.3).value),
            "underexposure_limit": float(self.declare_parameter("quality/underexposure_limit", 0.3).value),
            "overexposure_unusable": float(
                self.declare_parameter("quality/overexposure_unusable", 0.5).value
            ),
            "underexposure_unusable": float(
                self.declare_parameter("quality/underexposure_unusable", 0.5).value
            ),
        }

        self.enhancer_cfg = {
            "auto_clahe": bool(self.declare_parameter("enhancement/auto_clahe", True).value),
            "clahe_clip_limit": float(self.declare_parameter("enhancement/clahe_clip_limit", 2.0).value),
            "auto_gamma": bool(self.declare_parameter("enhancement/auto_gamma", True).value),
            "denoise_on_poor": bool(self.declare_parameter("enhancement/denoise_on_poor", True).value),
            "sharpen_on_blur": bool(self.declare_parameter("enhancement/sharpen_on_blur", True).value),
        }

        self.class_to_color_map = []
        class_map_param = self.declare_parameter("inference/class_to_color", []).value
        if isinstance(class_map_param, list):
            for idx, value in enumerate(class_map_param):
                if not isinstance(value, int):
                    self.get_logger().warn(
                        "[vision_node_py] inference/class_to_color[%d] is not int, ignored" % idx
                    )
                    continue
                if value < 0 or value > 5:
                    self.get_logger().warn(
                        "[vision_node_py] inference/class_to_color[%d]=%d out of range [0,5], "
                        "ignored" % (idx, value)
                    )
                    continue
                self.class_to_color_map.append(value)
            if self.class_to_color_map:
                self.get_logger().info(
                    "[vision_node_py] Loaded class remap entries: %d" % len(self.class_to_color_map)
                )
        else:
            self.get_logger().warn("[vision_node_py] inference/class_to_color must be an int array")

    def _init_backend(self):
        self.session = None
        self.backend_ready = False
        self.backend_name = "none"
        self.input_name = ""
        self.output_name = ""

        if self.backend_type == "fallback_only":
            self.backend_name = "fallback_hsv"
            return

        if self.backend_type != "onnx":
            if self.require_model:
                self.get_logger().fatal(
                    "[vision_node_py] Unsupported backend_type=%s with node/require_model=true"
                    % self.backend_type
                )
                raise RuntimeError("unsupported backend for required-model mode")
            self.get_logger().warn(
                "[vision_node_py] backend_type=%s unsupported, fallback_only" % self.backend_type
            )
            return

        if ort is None:
            if self.require_model:
                self.get_logger().fatal(
                    "[vision_node_py] onnxruntime not installed with node/require_model=true"
                )
                raise RuntimeError("onnxruntime missing")
            self.get_logger().warn("[vision_node_py] onnxruntime not installed, running fallback only")
            return

        if not self.model_path:
            if self.require_model:
                self.get_logger().fatal(
                    "[vision_node_py] inference/model_path is empty while node/require_model=true"
                )
                raise RuntimeError("model_path required but empty")
            self.get_logger().warn("[vision_node_py] model_path is empty, running fallback only")
            return

        providers = ["CPUExecutionProvider"]
        available = ort.get_available_providers()
        if "CUDAExecutionProvider" in available:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

        try:
            options = ort.SessionOptions()
            options.intra_op_num_threads = self.num_threads
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(
                self.model_path, sess_options=options, providers=providers
            )
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            self.backend_ready = True
            self.backend_name = "onnx"
        except Exception as exc:
            if self.require_model:
                self.get_logger().fatal("[vision_node_py] Failed to initialize ONNX backend: %s" % str(exc))
                raise
            self.get_logger().warn("[vision_node_py] Failed to initialize ONNX backend: %s" % str(exc))

    def _shutdown_worker(self):
        with self.latest_frame_cv:
            self.worker_shutdown = True
            self.latest_frame_cv.notify_all()
        if hasattr(self, "worker") and self.worker.is_alive():
            self.worker.join(timeout=1.0)

    def _image_callback(self, msg):
        with self.latest_frame_cv:
            now_mono = time.perf_counter()
            self.last_image_received_mono = now_mono
            self.latest_frame_buffer.store(
                PendingImageFrame(msg=msg, receive_mono=now_mono)
            )
            self.latest_frame_cv.notify()

    def _watchdog_cb(self, msg):
        """周期性检查图像 topic 心跳；若超时则发布 stale 诊断（避免 /diagnostics 沉默）。"""
        if self.last_image_received_mono <= 0.0:
            age_sec = float("inf")  # 从未收到图像
        else:
            age_sec = time.perf_counter() - self.last_image_received_mono

        if age_sec <= self.image_stale_threshold_sec:
            return  # 帧流正常，主流水线已经在发诊断

        status = DiagnosticStatus()
        status.name = "vision_node"
        status.hardware_id = "camera"
        status.level = DiagnosticStatus.ERROR
        if self.last_image_received_mono <= 0.0:
            status.message = "Vision watchdog: no image received yet on %s" % self.image_topic
            age_str = "inf"
        else:
            status.message = "Vision watchdog: image stream stale (%.2fs)" % age_sec
            age_str = "%.3f" % age_sec
        status.values = [
            KeyValue(key="state", value="WATCHDOG_STALE"),
            KeyValue(key="image_topic", value=self.image_topic),
            KeyValue(key="image_age_sec", value=age_str),
            KeyValue(
                key="image_stale_threshold_sec", value=str(self.image_stale_threshold_sec)
            ),
            KeyValue(key="frame_count", value=str(self.frame_count)),
            KeyValue(key="backend", value=self.backend_name),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        try:
            self.diag_pub_local.publish(arr)
            self.diag_pub_global.publish(arr)
        except Exception:
            pass  # 节点关停时正常忽略

    @staticmethod
    def _current_ros_time_sec():
        now_ros = rclpy.clock.Clock().now()
        now_sec = now_ros.nanoseconds / 1e9
        if now_sec > 0.0:
            return now_sec
        return time.time()

    def _worker_loop(self):
        while rclpy.ok():
            with self.latest_frame_cv:
                while not self.worker_shutdown and not self.latest_frame_buffer.has_pending():
                    self.latest_frame_cv.wait(timeout=0.1)
                if self.worker_shutdown:
                    return
                pick_mono = time.perf_counter()
                pending_frame = self.latest_frame_buffer.take_latest(now_mono=pick_mono)

            if pending_frame is None:
                if self.latest_frame_buffer.last_stale_age_ms > 0.0:
                    self.get_logger().warn(
                        "[vision_node_py] Dropped stale frame age_ms=%.1f total=%d"
                        % (self.latest_frame_buffer.last_stale_age_ms, self.latest_frame_buffer.stale_total),
                        throttle_duration_sec=2.0,
                    )
                continue

            self._process_pending_frame(pending_frame, pick_mono)

    def _process_pending_frame(self, pending_frame, pick_mono):
        try:
            bgr = self.bridge.imgmsg_to_cv2(pending_frame.msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(
                "[vision_node_py] cv_bridge exception: %s" % str(exc),
                throttle_duration_sec=5.0,
            )
            return

        self._process_frame(bgr, pending_frame.msg.header, pending_frame.receive_mono, pick_mono)

    def _run_model_inference(self, enhanced, bgr, quality, timing):
        """运行模型推理，返回 (detections, inference_us, newer_pending)。"""
        with self.latest_frame_cv:
            newer_pending_before = self.skip_heavy_if_newer_pending and self.latest_frame_buffer.has_pending()
        skip_for_freshness = newer_pending_before and self.fallback_enabled and self.backend_ready
        if skip_for_freshness:
            self.skipped_inference_newer_pending += 1
        skip_model = (
            skip_for_freshness
            or quality == QUALITY_UNUSABLE
            or self.state == STATE_VISION_LOST
            or self.state == STATE_FALLBACK
        )

        detections, inference_us = [], 0
        if not skip_model and self.backend_ready:
            t0 = time.perf_counter()
            detections = self._detect_onnx(enhanced, bgr.shape[1], bgr.shape[0])
            done_mono = time.perf_counter()
            inference_us = int((done_mono - t0) * 1e6)
            timing.mark_inference_done(done_mono)
        else:
            timing.mark_inference_done(time.perf_counter())

        with self.latest_frame_cv:
            newer_pending = newer_pending_before or (
                self.skip_heavy_if_newer_pending and self.latest_frame_buffer.has_pending()
            )
        if newer_pending:
            self.skipped_postprocess_newer_pending += 1
        return detections, inference_us, newer_pending

    def _run_fallback_detection(self, enhanced, quality):
        """运行 HSV fallback 检测，更新 last_fallback_ms，返回检测列表。"""
        need_fallback = self.fallback_enabled and (
            not self.backend_ready
            or quality == QUALITY_UNUSABLE
            or self.state in (STATE_VISION_LOST, STATE_FALLBACK)
        )
        t0 = time.perf_counter()
        detections = self._detect_fallback_hsv(enhanced) if need_fallback else []
        self.last_fallback_ms = (time.perf_counter() - t0) * 1000.0
        return detections

    def _process_frame(self, bgr, header, receive_mono, pick_mono):
        self.frame_count += 1
        timing = FrameStageTiming(header_stamp_sec=header.stamp.sec + header.stamp.nanosec * 1e-9, receive_mono=receive_mono)
        timing.mark_picked(pick_mono)
        quality_metrics = self._assess_quality(bgr)
        self._update_state(quality_metrics["overall"])

        enhanced = self._enhance_image(bgr, quality_metrics["overall"])
        timing.mark_preprocess_done(time.perf_counter())

        quality = quality_metrics["overall"]
        model_detections, inference_us, newer_pending = self._run_model_inference(enhanced, bgr, quality, timing)
        fallback_detections = self._run_fallback_detection(enhanced, quality)

        # 先对模型/HSV 各自的检测做 top-K，避免 fuse + tracker 处理 O(N^2) 大列表
        model_detections = self._filter_topk(model_detections, self.max_detections)
        fallback_detections = self._filter_topk(fallback_detections, self.max_detections)

        detections = self._fuse_detections(
            model_detections, fallback_detections, quality, self.state
        )

        _t0_tracker = time.perf_counter()
        if self.tracker_enabled and not newer_pending:
            detections = self.tracker.update(detections)
        self.last_tracker_ms = (time.perf_counter() - _t0_tracker) * 1000.0

        detections = self._filter_topk(detections, self.max_detections)
        timing.mark_postprocess_done(time.perf_counter())

        self._publish_detections(detections, header, quality_metrics, inference_us)
        if self.publish_debug_image and not newer_pending:
            self._publish_debug_image(enhanced, detections, header)
        timing.mark_publish_done(time.perf_counter(), self._current_ros_time_sec())
        self.last_stage_timing = timing.snapshot()
        self._publish_diagnostics(quality_metrics, len(detections), inference_us)

    def _assess_quality(self, bgr):
        metrics = {
            "blur_score": 0.0,
            "brightness": 0.0,
            "contrast": 0.0,
            "overexposure_ratio": 0.0,
            "underexposure_ratio": 0.0,
            "overall": QUALITY_UNUSABLE,
        }

        if bgr is None or bgr.size == 0:
            return metrics

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        lap = cv2.Laplacian(gray, cv2.CV_64F)
        _, sigma_lap = cv2.meanStdDev(lap)
        metrics["blur_score"] = float(sigma_lap[0][0] * sigma_lap[0][0])

        mu_gray, sigma_gray = cv2.meanStdDev(gray)
        metrics["brightness"] = float(mu_gray[0][0])
        metrics["contrast"] = float(sigma_gray[0][0])

        total = float(gray.size)
        if total > 0:
            metrics["overexposure_ratio"] = float(np.count_nonzero(gray > 240)) / total
            metrics["underexposure_ratio"] = float(np.count_nonzero(gray < 15)) / total

        t = self.quality_thresholds
        if (
            metrics["blur_score"] < t["blur_poor"]
            or metrics["overexposure_ratio"] > t["overexposure_unusable"]
            or metrics["underexposure_ratio"] > t["underexposure_unusable"]
            or metrics["brightness"] < t["brightness_very_low"]
            or metrics["brightness"] > t["brightness_very_high"]
        ):
            metrics["overall"] = QUALITY_UNUSABLE
        elif (
            metrics["blur_score"] < t["blur_degraded"]
            or metrics["brightness"] < t["brightness_low"]
            or metrics["brightness"] > t["brightness_high"]
            or metrics["overexposure_ratio"] > t["overexposure_limit"]
        ):
            metrics["overall"] = QUALITY_POOR
        elif metrics["blur_score"] < t["blur_good"] or metrics["contrast"] < 30.0:
            metrics["overall"] = QUALITY_DEGRADED
        else:
            metrics["overall"] = QUALITY_GOOD

        return metrics

    def _enhance_image(self, bgr, quality):
        if quality in (QUALITY_GOOD, QUALITY_UNUSABLE):
            return bgr  # no-copy: callers never modify the array in place

        out = bgr.copy()
        cfg = self.enhancer_cfg

        if cfg["auto_clahe"]:
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            channels = list(cv2.split(lab))
            clahe = cv2.createCLAHE(clipLimit=cfg["clahe_clip_limit"], tileGridSize=(8, 8))
            channels[0] = clahe.apply(channels[0])
            lab = cv2.merge(channels)
            out = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
        brightness = float(np.mean(gray))
        if cfg["auto_gamma"]:
            gamma = 1.0
            if brightness < 80.0:
                gamma = 0.6
            elif brightness > 180.0:
                gamma = 1.5
            if gamma != 1.0:
                lut = np.array(
                    [np.clip(((i / 255.0) ** gamma) * 255.0, 0.0, 255.0) for i in range(256)],
                    dtype=np.uint8,
                )
                out = cv2.LUT(out, lut)

        if quality == QUALITY_POOR:
            if cfg["denoise_on_poor"]:
                small = cv2.resize(out, (0, 0), fx=0.5, fy=0.5)
                denoised = cv2.bilateralFilter(small, 5, 50, 50)
                out = cv2.resize(denoised, (out.shape[1], out.shape[0]))
            if cfg["sharpen_on_blur"]:
                blurred = cv2.GaussianBlur(out, (0, 0), 2.0)
                out = cv2.addWeighted(out, 1.5, blurred, -0.5, 0)

        return out

    def _detect_fallback_hsv(self, bgr):
        if bgr is None or bgr.size == 0:
            return []

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        results = []
        kernels = (
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        )
        # HSV bounds are intentionally broad for degraded-light fallback.
        # 仅检测蓝/黄两类，红色/大黄锥靠模型；HSV 在低光下对红色误检率高，故不做 fallback。
        ranges = [
            (
                BLUE,
                np.array([90, 60, 60], dtype=np.uint8),
                np.array([140, 255, 255], dtype=np.uint8),
            ),
            (
                YELLOW,
                np.array([15, 60, 60], dtype=np.uint8),
                np.array([40, 255, 255], dtype=np.uint8),
            ),
        ]
        for color_type, lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernels[0])
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernels[1])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.fallback_min_area or area > self.fallback_max_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if w <= 0 or h <= 0:
                    continue
                aspect = float(w) / float(h)
                if aspect < 0.25 or aspect > 2.5:
                    continue
                fill_ratio = area / float(w * h)
                if fill_ratio < 0.2:
                    continue
                confidence = min(1.0, max(0.2, 0.3 * fill_ratio + 0.2))
                results.append(
                    Detection(
                        x=float(x) + float(w) * 0.5,
                        y=float(y) + float(h) * 0.5,
                        w=float(w),
                        h=float(h),
                        confidence=confidence,
                        class_id=int(color_type),
                        color_type=int(color_type),
                    )
                )
        return results

    @staticmethod
    def _fuse_detections(model_dets, fallback_dets, quality, state):
        if state in (STATE_VISION_LOST, STATE_FALLBACK):
            return fallback_dets

        if quality == QUALITY_GOOD and model_dets and state == STATE_NORMAL:
            return model_dets

        if quality == QUALITY_UNUSABLE:
            return fallback_dets

        merged = list(model_dets)
        for fallback_det in fallback_dets:
            overlap = False
            for model_det in model_dets:
                dx = fallback_det.x - model_det.x
                dy = fallback_det.y - model_det.y
                dist = float(np.hypot(dx, dy))
                avg_size = (model_det.w + model_det.h + fallback_det.w + fallback_det.h) * 0.25
                if dist < avg_size * 0.5:
                    overlap = True
                    break
            if not overlap:
                merged.append(fallback_det)
        return merged

    def _parse_onnx_nms_output(self, output, src_w, src_h):
        """解析 YOLOv8 nms=True 导出格式: [num_boxes, 6] (x1,y1,x2,y2,conf,class_id)。"""
        rows, cols = int(output.shape[0]), int(output.shape[1])
        if rows == 6 and cols != 6:
            output = output.T
            rows, cols = cols, rows
        scale_x = float(src_w) / float(self.input_width)
        scale_y = float(src_h) / float(self.input_height)
        detections = []
        for i in range(rows):
            conf = float(output[i, 4])
            if conf < self.conf_threshold:
                continue
            x1, y1 = float(output[i, 0]), float(output[i, 1])
            x2, y2 = float(output[i, 2]), float(output[i, 3])
            cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
            w, h = x2 - x1, y2 - y1
            cls = int(output[i, 5])
            detections.append(Detection(
                x=cx * scale_x, y=cy * scale_y, w=w * scale_x, h=h * scale_y,
                confidence=conf, class_id=cls,
                color_type=self._model_class_to_color_type(cls),
            ))
        return detections

    def _parse_onnx_raw_output(self, output, src_w, src_h):
        """解析 YOLOv8 nms=False 导出格式: [4+num_classes, num_anchors] 或其转置。"""
        rows, cols = int(output.shape[0]), int(output.shape[1])
        if rows > cols:
            output = output.T
            rows, cols = cols, rows
            self.get_logger().debug(
                "[vision_node_py] Transposed ONNX output to [%d, %d]" % (rows, cols)
            )
        if rows <= 4 or cols <= 0 or (rows - 4) <= 0:
            return []
        boxes, scores, class_ids = [], [], []
        for i in range(cols):
            cx, cy = float(output[0, i]), float(output[1, i])
            w, h = float(output[2, i]), float(output[3, i])
            cls_scores = output[4:, i]
            max_cls = int(np.argmax(cls_scores))
            max_score = float(cls_scores[max_cls])
            if max_score < self.conf_threshold:
                continue
            boxes.append([int(cx - w / 2.0), int(cy - h / 2.0), int(w), int(h)])
            scores.append(max_score)
            class_ids.append(max_cls)
        if not boxes:
            return []
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_threshold, self.nms_threshold)
        if indices is None or len(indices) == 0:
            return []
        scale_x = float(src_w) / float(self.input_width)
        scale_y = float(src_h) / float(self.input_height)
        detections = []
        for idx in np.array(indices).reshape(-1):
            box = boxes[int(idx)]
            cls = int(class_ids[int(idx)])
            detections.append(Detection(
                x=(box[0] + box[2] * 0.5) * scale_x, y=(box[1] + box[3] * 0.5) * scale_y,
                w=float(box[2]) * scale_x, h=float(box[3]) * scale_y,
                confidence=float(scores[int(idx)]), class_id=cls,
                color_type=self._model_class_to_color_type(cls),
            ))
        return detections

    def _detect_onnx(self, image_bgr, src_w, src_h):
        if not self.backend_ready or self.session is None:
            return []

        resized = cv2.resize(image_bgr, (self.input_width, self.input_height))
        blob = cv2.dnn.blobFromImage(
            resized,
            scalefactor=1.0 / 255.0,
            size=(self.input_width, self.input_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )

        try:
            outputs = self.session.run([self.output_name], {self.input_name: blob})
        except Exception as exc:
            self.get_logger().error(
                "[vision_node_py] ONNX inference failed: %s" % str(exc),
                throttle_duration_sec=2.0,
            )
            return []

        if not outputs:
            return []

        output = outputs[0]
        if output.ndim == 3 and output.shape[0] == 1:
            output = output[0]

        if output.ndim != 2:
            self.get_logger().error(
                "[vision_node_py] Unexpected ONNX output shape: %s" % str(list(output.shape)),
                throttle_duration_sec=2.0,
            )
            return []

        # 自动检测 ONNX 输出格式：
        #   A) YOLOv8 nms=True 导出: [num_boxes, 6]  (x1, y1, x2, y2, conf, class_id)
        #   B) YOLOv8 nms=False 导出: [num_anchors, 4+num_classes] 或转置后 [4+num_classes, num_anchors]
        rows, cols = int(output.shape[0]), int(output.shape[1])
        if cols == 6 or rows == 6:
            return self._parse_onnx_nms_output(output, src_w, src_h)
        return self._parse_onnx_raw_output(output, src_w, src_h)

    def _model_class_to_color_type(self, cls):
        """Map model class to color type.

        Vision outputs unified YELLOW (1), size classification done by LiDAR.
        """
        if self.class_to_color_map:
            if 0 <= cls < len(self.class_to_color_map):
                mapped = int(self.class_to_color_map[cls])
                if 0 <= mapped <= RED:
                    return mapped
            return NONE

        # Default map for vision model: 0=red, 1=blue, 2=yellow
        # Note: Vision outputs unified YELLOW, LiDAR classifies by size
        default_map = [RED, BLUE, YELLOW]
        if 0 <= cls < len(default_map):
            return default_map[cls]
        return NONE

    @staticmethod
    def _filter_topk(detections, max_det):
        if len(detections) <= max_det:
            return detections
        return sorted(detections, key=lambda d: d.confidence, reverse=True)[:max_det]

    def _publish_detections(self, detections, header, quality_metrics, inference_us):
        msg = HuatVisionDetections()
        msg.header = header
        msg.image_quality = int(quality_metrics["overall"])

        blur_score = float(quality_metrics["blur_score"])
        # blur_score 越大越清晰；blur_score=200 对应中位 0.5；blur_score<=0 视为最差。
        if blur_score <= 0.0:
            msg.quality_score = 0.0
        else:
            quality_score = 1.0 - (200.0 / blur_score) * 0.5
            msg.quality_score = max(0.0, min(1.0, quality_score))

        msg.x = [float(det.x) for det in detections]
        msg.y = [float(det.y) for det in detections]
        msg.color_types = [int(det.color_type) for det in detections]
        msg.confidences = [int(det.confidence * self.confidence_scale) for det in detections]
        msg.bbox_widths = [float(det.w) for det in detections]
        msg.bbox_heights = [float(det.h) for det in detections]

        using_fallback = (not self.backend_ready) or (
            quality_metrics["overall"] == QUALITY_UNUSABLE
        )
        msg.backend_name = self.backend_name
        msg.fallback_active = bool(using_fallback)
        msg.inference_time_us = int(max(inference_us, 0))

        self.detections_pub.publish(msg)

    def _publish_debug_image(self, bgr, detections, header):
        if self.debug_image_pub.get_subscription_count() == 0:
            return
        now = self.get_clock().now()
        if self.last_debug_pub > 0.0 and self.debug_image_rate > 0.0:
            min_interval = 1.0 / self.debug_image_rate
            if (now.nanoseconds / 1e9 - self.last_debug_pub) < min_interval:
                return
        self.last_debug_pub = now.nanoseconds / 1e9

        canvas = bgr.copy()
        for det in detections:
            color = self._COLOR_BGR.get(det.color_type, (200, 200, 200))

            x1 = int(det.x - det.w * 0.5)
            y1 = int(det.y - det.h * 0.5)
            x2 = int(det.x + det.w * 0.5)
            y2 = int(det.y + det.h * 0.5)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

        cv2.putText(
            canvas,
            self._state_to_string(self.state),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        try:
            out_msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            out_msg.header = header
            self.debug_image_pub.publish(out_msg)
        except CvBridgeError as exc:
            self.get_logger().error(
                "[vision_node_py] cv_bridge debug publish failed: %s" % str(exc),
                throttle_duration_sec=5.0,
            )

    def _build_diagnostic_kvs(self, quality_metrics, n_detections, inference_us):
        buf = self.latest_frame_buffer
        kvs = [
            KeyValue(key="state", value=self._state_to_string(self.state)),
            KeyValue(key="n_detections", value=str(n_detections)),
            KeyValue(key="inference_time_us", value=str(inference_us)),
            KeyValue(key="image_quality", value=str(int(quality_metrics["overall"]))),
            KeyValue(key="blur_score", value=str(float(quality_metrics["blur_score"]))),
            KeyValue(key="brightness", value=str(float(quality_metrics["brightness"]))),
            KeyValue(key="frame_count", value=str(self.frame_count)),
            KeyValue(key="backend", value=self.backend_name),
            KeyValue(key="require_model", value="1" if self.require_model else "0"),
            KeyValue(key="tracker_enabled", value="1" if self.tracker_enabled else "0"),
            KeyValue(key="fallback_enabled", value="1" if self.fallback_enabled else "0"),
            KeyValue(key="model_path", value=self.model_path if self.model_path else "<empty>"),
            KeyValue(key="stale_frame_age_sec", value=str(self.stale_frame_age_sec)),
            KeyValue(key="frame_drop_replaced_total", value=str(buf.replaced_total)),
            KeyValue(key="frame_drop_stale_total", value=str(buf.stale_total)),
            KeyValue(key="pending_frame_depth", value=str(buf.pending_depth())),
            KeyValue(key="last_stale_age_ms", value=str(buf.last_stale_age_ms)),
            KeyValue(key="skipped_inference_newer_pending", value=str(self.skipped_inference_newer_pending)),
            KeyValue(key="skipped_postprocess_newer_pending", value=str(self.skipped_postprocess_newer_pending)),
            KeyValue(key="skip_heavy_if_newer_pending", value="1" if self.skip_heavy_if_newer_pending else "0"),
            KeyValue(key="fallback_ms", value=str(self.last_fallback_ms)),
            KeyValue(key="tracker_ms", value=str(self.last_tracker_ms)),
        ]
        t = self.last_stage_timing or {}
        for k in ("receive_to_pick_ms", "preprocess_ms", "inference_stage_ms",
                  "postprocess_ms", "publish_ms", "total_processing_ms", "end_to_end_publish_lag_ms"):
            kvs.append(KeyValue(key=k, value=str(t.get(k, 0.0))))
        return kvs

    def _publish_diagnostics(self, quality_metrics, n_detections, inference_us):
        now = self.get_clock().now()
        if self.last_diag_pub > 0.0 and self.diag_rate_hz > 0.0:
            if (now.nanoseconds / 1e9 - self.last_diag_pub) < 1.0 / self.diag_rate_hz:
                return
        self.last_diag_pub = now.nanoseconds / 1e9

        level, message = self._STATE_DIAG.get(
            self.state, (DiagnosticStatus.ERROR, "Vision lost - image unusable")
        )

        status = DiagnosticStatus()
        status.name = "vision_node"
        status.hardware_id = "camera"
        status.level = level
        status.message = message
        status.values = self._build_diagnostic_kvs(quality_metrics, n_detections, inference_us)

        arr = DiagnosticArray()
        arr.header.stamp = now.to_msg()
        arr.status = [status]

        self.diag_pub_local.publish(arr)
        self.diag_pub_global.publish(arr)

    def _update_state(self, quality):
        _counter_for = {
            QUALITY_GOOD: "consecutive_good",
            QUALITY_DEGRADED: "consecutive_degraded",
            QUALITY_POOR: "consecutive_poor",
        }
        _all_counters = ["consecutive_good", "consecutive_degraded", "consecutive_poor", "consecutive_unusable"]
        active = _counter_for.get(quality, "consecutive_unusable")
        for attr in _all_counters:
            setattr(self, attr, getattr(self, attr) + 1 if attr == active else 0)

        previous = self.state
        for counter_attr, threshold_attr, next_state in _STATE_TRANSITIONS.get(self.state, []):
            if getattr(self, counter_attr) >= getattr(self, threshold_attr):
                self.state = next_state
                break

        if previous != self.state:
            self.get_logger().info(
                "[vision_node_py] State transition: %s -> %s"
                % (self._state_to_string(previous), self._state_to_string(self.state))
            )

    @staticmethod
    def _state_to_string(state):
        if state == STATE_NORMAL:
            return "NORMAL"
        if state == STATE_DEGRADED:
            return "DEGRADED"
        if state == STATE_FALLBACK:
            return "FALLBACK"
        if state == STATE_VISION_LOST:
            return "VISION_LOST"
        return "UNKNOWN"


def main(args=None):
    rclpy.init(args=args)
    node = VisionNodePy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
