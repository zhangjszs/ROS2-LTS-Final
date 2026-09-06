#!/usr/bin/env python3
"""
LiDAR 聚类性能采集器
订阅 /debug/lidar_cluster/profiling 并打印实时统计信息。
与 sensor_stack.launch 一起运行：
    ros2 run lidar_cluster profile_collector.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from collections import deque


class ProfileCollector(Node):
    def __init__(self, window=100):
        super().__init__('lidar_profile_collector')
        self.window = window
        self.pt = deque(maxlen=window)
        self.seg = deque(maxlen=window)
        self.cluster = deque(maxlen=window)
        self.total = deque(maxlen=window)
        self.count = 0
        self.first_msg = True

        self.declare_parameter('window', window)
        self.window = self.get_parameter('window').get_parameter_value().integer_value
        self.pt = deque(maxlen=self.window)
        self.seg = deque(maxlen=self.window)
        self.cluster = deque(maxlen=self.window)
        self.total = deque(maxlen=self.window)

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/debug/lidar_cluster/profiling',
            self.on_profile,
            10,
        )
        self.get_logger().info('[profile_collector] Started, waiting for /debug/lidar_cluster/profiling ...')
        self.get_logger().info('[profile_collector] Tip: make sure enable_profiling=true in lidar_cluster.yaml')

    def on_profile(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn('[profile_collector] Received msg with < 4 fields, ignoring', once=True)
            return

        pt, seg, cluster, total = msg.data[0], msg.data[1], msg.data[2], msg.data[3]
        self.pt.append(pt)
        self.seg.append(seg)
        self.cluster.append(cluster)
        self.total.append(total)
        self.count += 1

        if self.first_msg:
            self.get_logger().info('[profile_collector] First message received! Collecting...')
            self.first_msg = False

        # 每10帧打印简化实时视图（10Hz LiDAR 下为 1Hz）
        if self.count % 10 == 0:
            self.get_logger().info(
                '[profile_collector] Live frame %d | PT=%.2f SEG=%.2f CLU=%.2f TOT=%.2f ms',
                self.count, pt, seg, cluster, total,
            )

        # 每50帧打印滚动统计信息
        if self.count % 50 == 0:
            self.print_stats()

    def print_stats(self):
        def stats(d):
            if not d:
                return 0.0, 0.0, 0.0
            return min(d), sum(d) / len(d), max(d)

        pt_min, pt_avg, pt_max = stats(self.pt)
        seg_min, seg_avg, seg_max = stats(self.seg)
        cl_min, cl_avg, cl_max = stats(self.cluster)
        tot_min, tot_avg, tot_max = stats(self.total)

        self.get_logger().info(
            "\n========== Profiling Stats (last %d frames) ==========\n"
            "  PassThrough  min=%6.2f  avg=%6.2f  max=%6.2f  ms\n"
            "  GroundSeg    min=%6.2f  avg=%6.2f  max=%6.2f  ms\n"
            "  Cluster      min=%6.2f  avg=%6.2f  max=%6.2f  ms\n"
            "  Total        min=%6.2f  avg=%6.2f  max=%6.2f  ms\n"
            "=====================================================",
            len(self.total),
            pt_min, pt_avg, pt_max,
            seg_min, seg_avg, seg_max,
            cl_min, cl_avg, cl_max,
            tot_min, tot_avg, tot_max,
        )


def main(args=None):
    rclpy.init(args=args)
    window = 100
    collector = ProfileCollector(window=window)
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
