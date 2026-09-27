# #33：vision 质量状态机迁移表 + 质量阶梯 + 融合选择 + 时序去抖（pytest）
#
# 钉住三类行为，改阈值必须同步改测试（防静默漂移）：
#   1. 状态机迁移表：每个阈值穿越方向可断言（默认 degraded 3 / poor 5 /
#      unusable 10 / recovery 5，见 VisionNodePy._load_params）。
#   2. 质量阶梯：合成图像（棋盘格 + 高斯模糊 / 全黑 / 全白）穿越阈值。
#   3. ONNX 缺席时 HSV 接管选择（_fuse_detections）与 IoU 时序去抖
#     （TemporalTracker min_hits / max_miss）。
# 运行：source /opt/ros/<distro>/setup.bash && source install/setup.bash &&
#   python3 -m pytest src/vision_ros/test/test_vision_quality.py

import numpy as np

from vision_ros.vision_node_py import (
    QUALITY_DEGRADED,
    QUALITY_GOOD,
    QUALITY_POOR,
    QUALITY_UNUSABLE,
    STATE_DEGRADED,
    STATE_FALLBACK,
    STATE_NORMAL,
    STATE_VISION_LOST,
    Detection,
    LatestFrameBuffer,
    TemporalTracker,
    VisionNodePy,
)

# 与 VisionNodePy._load_params 默认值保持一致；改默认值必须同步改这里。
DEFAULT_THRESHOLDS = {
    "blur_good": 200.0,
    "blur_degraded": 100.0,
    "blur_poor": 50.0,
    "brightness_low": 40.0,
    "brightness_high": 220.0,
    "brightness_very_low": 15.0,
    "brightness_very_high": 250.0,
    "overexposure_limit": 0.3,
    "underexposure_limit": 0.3,
    "overexposure_unusable": 0.5,
    "underexposure_unusable": 0.5,
}


def _bare_node():
    """不起 ROS 节点的裸对象：只装 _update_state/_assess_quality 所需属性。"""
    node = VisionNodePy.__new__(VisionNodePy)
    node.state = STATE_NORMAL
    node.consecutive_good = 0
    node.consecutive_degraded = 0
    node.consecutive_poor = 0
    node.consecutive_unusable = 0
    node.degraded_frame_count = 3
    node.poor_frame_count = 5
    node.unusable_frame_count = 10
    node.recovery_frame_count = 5
    node.quality_thresholds = dict(DEFAULT_THRESHOLDS)

    class _Logger:
        def info(self, msg):
            pass

    node.get_logger = lambda: _Logger()
    return node


def _checkerboard():
    # 取值 40/200：避开 0/255 极值，免得过曝/欠曝比恰好卡在阈值上。
    return (np.indices((480, 640)).sum(axis=0) // 40 % 2 * 160 + 40).astype(np.uint8)


def _overexposed_partial():
    # 120 底 + 约 40% 白块：blur 高、过曝比 0.4（(0.3, 0.5]），落 POOR。
    idx = np.indices((480, 640))
    mask = idx.sum(axis=0) // 80 % 5 < 2
    return np.where(mask, 255, 120).astype(np.uint8)


def _bgr(gray):
    import cv2

    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


# ── 质量阶梯 ────────────────────────────────────────────────────────────────

class TestQualityLadder:
    def test_sharp_is_good(self):
        node = _bare_node()
        assert node._assess_quality(_bgr(_checkerboard()))["overall"] == QUALITY_GOOD

    def test_mild_blur_is_degraded(self):
        # blur≈113，落在 [blur_degraded=100, blur_good=200)。
        import cv2

        node = _bare_node()
        blurred = cv2.GaussianBlur(_checkerboard(), (5, 5), 0)
        assert node._assess_quality(_bgr(blurred))["overall"] == QUALITY_DEGRADED

    def test_partial_overexposure_is_poor(self):
        node = _bare_node()
        assert node._assess_quality(_bgr(_overexposed_partial()))["overall"] == QUALITY_POOR

    def test_heavy_blur_is_unusable(self):
        # blur≈4.6 < blur_poor=50。
        import cv2

        node = _bare_node()
        blurred = cv2.GaussianBlur(_checkerboard(), (31, 31), 0)
        assert node._assess_quality(_bgr(blurred))["overall"] == QUALITY_UNUSABLE

    def test_dark_is_unusable(self):
        node = _bare_node()
        dark = np.full((480, 640), 5, dtype=np.uint8)
        assert node._assess_quality(_bgr(dark))["overall"] == QUALITY_UNUSABLE

    def test_white_is_unusable(self):
        node = _bare_node()
        white = np.full((480, 640), 255, dtype=np.uint8)
        assert node._assess_quality(_bgr(white))["overall"] == QUALITY_UNUSABLE


# ── 状态机迁移 ──────────────────────────────────────────────────────────────

class TestStateMachine:
    def _feed(self, node, quality, n):
        for _ in range(n):
            node._update_state(quality)
        return node.state

    def test_normal_to_degraded_on_third_degraded(self):
        node = _bare_node()
        assert self._feed(node, QUALITY_DEGRADED, 2) == STATE_NORMAL
        assert self._feed(node, QUALITY_DEGRADED, 1) == STATE_DEGRADED

    def test_normal_to_fallback_on_fifth_poor(self):
        node = _bare_node()
        assert self._feed(node, QUALITY_POOR, 4) == STATE_NORMAL
        assert self._feed(node, QUALITY_POOR, 1) == STATE_FALLBACK

    def test_normal_unusable_has_priority_over_poor(self):
        # 优先级：unusable > poor。4 帧 poor 未达门限时来 10 帧 unusable，
        # 应进 VISION_LOST 而非 FALLBACK。
        node = _bare_node()
        self._feed(node, QUALITY_POOR, 4)
        assert self._feed(node, QUALITY_UNUSABLE, 10) == STATE_VISION_LOST

    def test_fallback_recovers_to_normal_on_good(self):
        node = _bare_node()
        self._feed(node, QUALITY_POOR, 5)
        assert node.state == STATE_FALLBACK
        assert self._feed(node, QUALITY_GOOD, 5) == STATE_NORMAL

    def test_vision_lost_to_fallback_on_poor(self):
        node = _bare_node()
        self._feed(node, QUALITY_UNUSABLE, 10)
        assert node.state == STATE_VISION_LOST
        assert self._feed(node, QUALITY_POOR, 5) == STATE_FALLBACK

    def test_vision_lost_to_degraded_on_degraded(self):
        node = _bare_node()
        self._feed(node, QUALITY_UNUSABLE, 10)
        assert node.state == STATE_VISION_LOST
        assert self._feed(node, QUALITY_DEGRADED, 5) == STATE_DEGRADED

    def test_fallback_degrades_on_degraded(self):
        node = _bare_node()
        self._feed(node, QUALITY_POOR, 5)
        assert node.state == STATE_FALLBACK
        assert self._feed(node, QUALITY_DEGRADED, 5) == STATE_DEGRADED


# ── 融合选择（ONNX 缺席时 HSV 接管） ─────────────────────────────────────────

def _det(x=100.0, y=100.0):
    return Detection(x, y, 40.0, 40.0, 0.9, 1, 1)


class TestFusionSelection:
    def test_vision_lost_uses_fallback(self):
        fb = [_det()]
        assert VisionNodePy._fuse_detections([], fb, QUALITY_UNUSABLE, STATE_VISION_LOST) == fb

    def test_fallback_state_uses_fallback(self):
        model, fb = [_det(100.0, 100.0)], [_det(300.0, 300.0)]
        assert VisionNodePy._fuse_detections(model, fb, QUALITY_POOR, STATE_FALLBACK) == fb

    def test_normal_good_uses_model(self):
        model, fb = [_det(100.0, 100.0)], [_det(300.0, 300.0)]
        assert VisionNodePy._fuse_detections(model, fb, QUALITY_GOOD, STATE_NORMAL) == model

    def test_unusable_quality_uses_fallback(self):
        model, fb = [_det(100.0, 100.0)], [_det(300.0, 300.0)]
        assert VisionNodePy._fuse_detections(model, fb, QUALITY_UNUSABLE, STATE_NORMAL) == fb

    def test_degraded_merges_non_overlapping(self):
        model, fb = [_det(100.0, 100.0)], [_det(300.0, 300.0)]
        merged = VisionNodePy._fuse_detections(model, fb, QUALITY_DEGRADED, STATE_DEGRADED)
        assert len(merged) == 2

    def test_degraded_dedups_overlapping(self):
        model, fb = [_det(100.0, 100.0)], [_det(105.0, 105.0)]
        merged = VisionNodePy._fuse_detections(model, fb, QUALITY_DEGRADED, STATE_DEGRADED)
        assert len(merged) == 1


# ── 时序去抖 ────────────────────────────────────────────────────────────────

class TestTemporalTracker:
    def test_single_frame_suppressed_with_min_hits_two(self):
        tracker = TemporalTracker(iou_threshold=0.4, max_miss=1, min_hits=2)
        assert tracker.update([_det()]) == []
        assert len(tracker.update([_det()])) == 1

    def test_flicker_never_confirms(self):
        tracker = TemporalTracker(iou_threshold=0.4, max_miss=1, min_hits=2)
        for _ in range(4):
            tracker.update([_det()])
            tracker.update([])
        assert tracker.update([]) == []

    def test_expired_after_max_miss(self):
        tracker = TemporalTracker(iou_threshold=0.4, max_miss=1, min_hits=2)
        tracker.update([_det()])
        tracker.update([_det()])
        tracker.update([])
        tracker.update([])
        assert tracker.update([_det()]) == []


# ── 最新帧缓冲 ──────────────────────────────────────────────────────────────

class TestLatestFrameBuffer:
    def test_take_latest_returns_fresh_frame(self):
        buf = LatestFrameBuffer(stale_after_sec=0.5)

        class _Frame:
            receive_mono = 100.0

        frame = _Frame()
        buf.store(frame)
        assert buf.has_pending()
        assert buf.take_latest(100.2) is frame
        assert not buf.has_pending()

    def test_stale_frame_dropped(self):
        buf = LatestFrameBuffer(stale_after_sec=0.5)

        class _Frame:
            receive_mono = 100.0

        buf.store(_Frame())
        assert buf.take_latest(101.0) is None
        assert buf.stale_total == 1

    def test_replace_counts(self):
        buf = LatestFrameBuffer(stale_after_sec=0.0)

        class _Frame:
            receive_mono = 0.0

        buf.store(_Frame())
        buf.store(_Frame())
        assert buf.replaced_total == 1
