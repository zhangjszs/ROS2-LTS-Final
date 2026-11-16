#!/usr/bin/env python3
"""#107：真实 ROS Topic/QoS、生产监控和仲裁出口；无 PP watchdog 掩盖定位断流。"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
try:
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from common_msgs.msg import HuatCarstate, HuatPathLimits, HuatStop, HuatSystemState, HuatVehicleCmd
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
except ImportError as exc:
    (OUT / "summary.json").write_text(json.dumps({"status": "NOT_RUN", "reason": str(exc)}, indent=2))
    print(f"NOT_RUN: {exc}; source ROS 2 Jazzy and install/setup.bash", flush=True)
    sys.exit(2)

STATE_TIMEOUT, PATH_TIMEOUT = 0.3, 0.5
# DDS/调度容差单独声明，绝不当作实车停车预算。正常检测 20ms + 心跳 100ms。
JITTER = 0.35


class Harness(Node):
    def __init__(self):
        super().__init__("vehicle_state_lease_harness")
        self.started = time.monotonic()
        self.timeline = []
        self.cases = []
        self.processes = {}
        self.logs = []
        self.stop = self.state = self.command = None
        self.monitor_inputs = None
        self.stop_at = self.state_at = self.command_at = -math.inf
        self.sim = 100.0
        self.clock_runs = True
        self.state_runs = False
        self.path_runs = True
        self.invalid_field = None
        self.path_kind = "valid"
        self.last_vehicle_rx = self.last_path_rx = None
        self.next_path = self.next_state = self.started
        stop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(HuatStop, "/system/stop", self.on_stop, stop_qos)
        self.create_subscription(HuatSystemState, "/system/state", self.on_state, 10)
        self.create_subscription(HuatVehicleCmd, "/vehicle_command", self.on_command, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
        self.pub_vehicle = self.create_publisher(HuatCarstate, "/localization/vehicle_state", 10)
        self.pub_path = self.create_publisher(HuatPathLimits, "/planning/pathlimits", 1)
        self.pub_command = self.create_publisher(HuatVehicleCmd, "/mpc/vehicle_command", 10)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_event = self.create_publisher(String, "/system/task_event", 10)
        self.pub_request = self.create_publisher(HuatStop, "/planning/track/stop_request", 10)
        self.pub_reset = self.create_publisher(HuatStop, "/system/reset_stop", 1)
        self.last_tick = self.started
        self.create_timer(0.02, self.tick)

    def record(self, event, **values):
        self.timeline.append({"at_steady_sec": time.monotonic() - self.started,
                              "ros_sec": self.sim, "event": event, **values})

    def stamp(self, stamp):
        stamp.sec = int(self.sim)
        stamp.nanosec = int((self.sim - int(self.sim)) * 1e9)

    def vehicle(self):
        m = HuatCarstate()
        self.stamp(m.header.stamp)
        m.header.frame_id = "map"
        if self.invalid_field == "v":
            m.v = float("nan")
        elif self.invalid_field:
            setattr(m.car_state, self.invalid_field, float("nan"))
        self.pub_vehicle.publish(m)
        self.last_vehicle_rx = time.monotonic() - self.started
        self.record("vehicle_input", valid=self.invalid_field is None, field=self.invalid_field)

    def tick(self):
        now = time.monotonic()
        if self.clock_runs:
            self.sim += now - self.last_tick
        self.last_tick = now
        c = Clock()
        self.stamp(c.clock)
        self.pub_clock.publish(c)
        if self.state_runs and now >= self.next_state:
            self.vehicle()
            self.next_state = now + 0.05
        if self.path_runs and now >= self.next_path:
            m = HuatPathLimits()
            self.stamp(m.header.stamp)
            m.header.frame_id = "map"
            m.path = [Point(x=0.0), Point(x=1.0)]
            if self.path_kind == "empty":
                m.path = []
            elif self.path_kind == "future":
                m.header.stamp.sec += 10
            elif self.path_kind == "old":
                m.header.stamp.sec -= 10
            self.pub_path.publish(m)
            self.last_path_rx = now - self.started
            self.next_path = now + 0.1
            self.record("path_input", kind=self.path_kind)
        # 持续提供新鲜、校验合法且含正油门的 MPC 源；保护必须来自系统监控。
        m = HuatVehicleCmd()
        m.head1, m.head2, m.length = 0xAA, 0x55, 10
        m.steering, m.brake_force, m.pedal_ratio = 90, 0, 30
        m.gear_position, m.working_mode, m.racing_num, m.racing_status = 1, 1, 1, 1
        m.checksum = sum([90, 0, 30, 1, 1, 1, 1]) & 0xFFFF
        self.pub_command.publish(m)

    def on_stop(self, m):
        self.stop, self.stop_at = bool(m.stop), time.monotonic()
        self.record("system_stop", stop=self.stop)

    def on_state(self, m):
        self.state, self.state_at = m, time.monotonic()
        self.record("system_state", stop=bool(m.stop_active), task=int(m.task_state),
                    kind=int(m.stop_kind), can_drive=bool(m.can_drive), pedal=int(m.pedal_ratio))

    def on_command(self, m):
        self.command, self.command_at = m, time.monotonic()
        self.record("vehicle_command", pedal=int(m.pedal_ratio), brake=int(m.brake_force))

    def on_diagnostics(self, m):
        for status in m.status:
            values = {v.key: v.value for v in status.values}
            if "Steady check time (s)" not in values:
                continue
            checked = float(values["Steady check time (s)"])
            vehicle_age = float(values["Vehicle receive age (steady s)"])
            planner_age = float(values["Planner receive age (steady s)"])
            # 同一 Linux 主机的 C++ steady_clock / Python monotonic 共用时钟域。
            # 从节点检查时刻及节点报告年龄还原最后合法接收，不用 harness 的发送时刻冒充。
            self.monitor_inputs = {
                "checked_at_steady_sec": checked - self.started,
                "vehicle_last_rx_steady_sec": checked - vehicle_age - self.started if math.isfinite(vehicle_age) else None,
                "path_last_rx_steady_sec": checked - planner_age - self.started if math.isfinite(planner_age) else None,
                "vehicle_freshness": values.get("Vehicle freshness"),
                "planner_freshness": values.get("Planner freshness"),
            }
            self.record("monitor_input_leases", **self.monitor_inputs)

    def pump(self, duration, invariant=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            for name, process in self.processes.items():
                assert process.poll() is None, f"{name} exited rc={process.returncode}"
            if invariant:
                assert invariant(), "safety invariant violated during sustained observation"

    def wait(self, predicate, budget=3.0):
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline:
            self.pump(0.01)
            if predicate():
                return
        raise AssertionError(f"condition timed out after {budget}s")

    def safe(self):
        return self.stop is True and self.stop_at > time.monotonic() - 0.3 and self.braked()

    def braked(self):
        now = time.monotonic()
        return (self.state is not None and self.state.stop_active
                and not self.state.can_drive and self.command is not None
                and self.command.pedal_ratio == 0 and self.command.brake_force > 0
                and min(self.state_at, self.command_at) > now - 0.3)

    def ready(self):
        return self.stop is False and self.state is not None and not self.state.stop_active

    def event(self, event):
        self.record("task_event", value=event)
        self.pub_event.publish(String(data=event))
        self.pump(0.08)

    def event_until(self, event, acknowledged, budget=3.0):
        # 不同 Topic 无跨通道顺序保证：harness 先看到 stop=false，不代表仲裁器已处理。
        # 显式命令按既有幂等语义重发，只有状态遥测确认才成功；不用固定 sleep 猜调度。
        deadline = time.monotonic() + budget
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.record("task_event", value=event, retry_until_ack=True)
                self.pub_event.publish(String(data=event))
                next_send = now + 0.1
            self.pump(0.01)
            if acknowledged():
                return
        raise AssertionError(f"task event {event} not acknowledged after {budget}s")

    def drive(self):
        self.wait(self.ready)
        self.event("arm")
        self.event("start")
        self.wait(lambda: self.command is not None and self.command.pedal_ratio > 0
                  and self.state.can_drive)

    def start(self, name):
        exe = "safety_monitor" if name == "monitor" else "command_arbiter_node"
        path = Path(get_package_prefix("safety_monitor")) / "lib/safety_monitor" / exe
        args = [str(path), "--ros-args", "-p", "use_sim_time:=true"]
        if name == "monitor":
            args += ["-p", f"safety.state_timeout_sec:={STATE_TIMEOUT}", "-p", f"planner_timeout:={PATH_TIMEOUT}",
                     "-p", "stop_request_topic:=/planning/track/stop_request",
                     "-p", "reset_stop_topic:=/system/reset_stop"]
        else:
            args += ["-p", "task.autostart:=false"]
        log = (OUT / f"{name}_{len(self.logs)}.log").open("w")
        self.logs.append(log)
        self.processes[name] = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.record("process_start", name=name, pid=self.processes[name].pid)

    def kill(self, name):
        p = self.processes.pop(name)
        os.killpg(p.pid, signal.SIGKILL)
        p.wait(timeout=3)
        self.record("process_kill", name=name)

    def case(self, name, action):
        self.record("trigger", case=name, last_vehicle_input=self.last_vehicle_rx, last_path_input=self.last_path_rx,
                    monitor_inputs=self.monitor_inputs)
        start = len(self.timeline) - 1
        try:
            action()
            self.cases.append({"case": name, "status": "PASS", "timeline_start": start,
                               "timeline_end": len(self.timeline) - 1})
            print(f"PASS {name}", flush=True)
        except Exception as exc:
            self.record("failure_snapshot", monitor_stop=self.stop,
                        monitor_receive_age_sec=time.monotonic() - self.stop_at if math.isfinite(self.stop_at) else None,
                        task=None if self.state is None else int(self.state.task_state),
                        stop_kind=None if self.state is None else int(self.state.stop_kind),
                        can_drive=None if self.state is None else bool(self.state.can_drive),
                        pedal=None if self.command is None else int(self.command.pedal_ratio),
                        brake=None if self.command is None else int(self.command.brake_force),
                        monitor_inputs=self.monitor_inputs)
            print("FAIL snapshot: " + json.dumps(self.timeline[-1]), flush=True)
            self.cases.append({"case": name, "status": "FAIL", "reason": str(exc), "timeline_start": start})
            raise
        finally:
            self.save()

    def save(self, status="RUNNING", reason=None):
        (OUT / "timeline.json").write_text(json.dumps({"clock": "steady receive / ROS publication", "events": self.timeline}, indent=2))
        (OUT / "summary.json").write_text(json.dumps({"status": status, "reason": reason,
            "state_timeout_sec": STATE_TIMEOUT, "path_timeout_sec": PATH_TIMEOUT,
            "software_jitter_budget_sec": JITTER, "cases": self.cases}, indent=2))

    def close(self):
        for name in list(self.processes):
            self.kill(name)
        for log in self.logs:
            log.close()


def run(h):
    h.start("monitor")
    h.start("arbiter")
    h.wait(lambda: h.pub_vehicle.get_subscription_count() > 0 and h.pub_path.get_subscription_count() > 0
           and h.pub_command.get_subscription_count() > 0 and h.pub_event.get_subscription_count() > 0)

    def absent():
        h.wait(h.safe)
        h.pump(0.4, h.safe)
    h.case("startup_no_vehicle", absent)

    def single_frame():
        h.vehicle()  # 只有一帧，路径继续 10Hz；先证明初始 TIMEOUT 能清除。
        h.wait(lambda: h.ready(), 0.25)
        h.wait(h.safe, STATE_TIMEOUT + JITTER)
        h.pump(0.4, h.safe)
    h.case("one_vehicle_frame_continuous_path", single_frame)

    def no_authorization():
        h.state_runs = True
        h.wait(h.ready)
        h.event("start")  # 缺 ARM
        h.pump(0.2, lambda: h.command.pedal_ratio == 0 and not h.state.can_drive)
        assert h.command.pedal_ratio == 0 and not h.state.can_drive
        h.event("arm")  # 缺 START
        h.pump(0.2, lambda: h.command.pedal_ratio == 0 and not h.state.can_drive)
        assert h.command.pedal_ratio == 0 and not h.state.can_drive
        h.event("start")
        h.wait(lambda: h.command.pedal_ratio > 0 and h.state.can_drive)
    h.case("healthy_and_explicit_arm_start", no_authorization)

    def lost(state, path):
        h.state_runs, h.path_runs = not state, not path
        h.wait(h.safe, (STATE_TIMEOUT if state else PATH_TIMEOUT) + JITTER)
        h.pump(0.4, h.safe)
    h.case("vehicle_only_lost_while_driving", lambda: lost(True, False))
    h.state_runs = True
    h.drive()
    h.case("path_only_lost_while_driving", lambda: lost(False, True))
    h.path_runs = True
    h.drive()
    h.case("both_lost", lambda: lost(True, True))

    def recovery(path_first):
        h.state_runs = h.path_runs = False
        # 两种租约期限不同；仅观察到 STOP 不能证明两源都已过期。
        # 先建立双源过期前提，再恢复其中一个，避免尚新鲜的另一租约造成假失败。
        h.pump(max(STATE_TIMEOUT, PATH_TIMEOUT) + JITTER)
        h.wait(h.safe, PATH_TIMEOUT + JITTER)
        if path_first:
            h.path_runs = True
        else:
            h.state_runs = True
        h.pump(0.7, h.safe)
        h.state_runs = h.path_runs = True
        h.wait(h.ready)
    h.case("path_recovers_before_vehicle", lambda: recovery(True))
    h.case("vehicle_recovers_before_path", lambda: recovery(False))

    for field in ("x", "y", "theta", "v"):
        def invalid(field=field):
            h.drive()
            h.invalid_field = field
            h.wait(h.safe, STATE_TIMEOUT + JITTER)
            h.pump(0.4, h.safe)
            h.invalid_field = None
            h.wait(h.ready)
        h.case(f"nonfinite_vehicle_{field}", invalid)

    for kind in ("empty", "old", "future"):
        def invalid_path(kind=kind):
            h.drive()
            h.path_kind = kind
            h.wait(h.safe, PATH_TIMEOUT + JITTER)
            h.pump(0.3, h.safe)
            h.path_kind = "valid"
            h.wait(h.ready)
        h.case(f"invalid_path_{kind}", invalid_path)

    def request():
        h.pub_request.publish(HuatStop(stop=True))
        h.wait(h.safe)
        h.pump(0.5, h.safe)
        assert h.state.stop_kind == HuatSystemState.STOPKIND_REQUEST
        h.pub_request.publish(HuatStop(stop=False))
        h.pump(0.1)
        h.pub_reset.publish(HuatStop(stop=False))
        h.wait(lambda: h.stop is False)
        # 现有任务契约：REQUEST 的显式 reset 保留先前 RUNNING；健康数据本身不能清锁。
        h.event_until("reset", lambda: h.ready() and h.state.can_drive and h.command.pedal_ratio > 0)
        assert h.state.task_state == HuatSystemState.TASK_RUNNING
        # FINISHED / FAULT 的 reset 则回 IDLE，必须重新 ARM→START 才可起步。
        h.event("finish")
        h.wait(h.braked)
        h.event_until("reset", h.ready)
        assert h.state.task_state == HuatSystemState.TASK_IDLE
        assert not h.state.can_drive and h.command.pedal_ratio == 0
    h.case("request_latch_survives_healthy_data", request)

    def fault():
        h.event("abort")
        h.wait(h.braked)
        h.pump(0.5, h.braked)
        assert h.state.stop_kind == HuatSystemState.STOPKIND_FAULT
        h.event_until("reset", h.ready)
        assert not h.state.can_drive and h.command.pedal_ratio == 0
    h.case("fault_latch_survives_healthy_data", fault)

    def arbiter_restart():
        h.drive()
        h.kill("arbiter")
        h.command = h.state = None
        h.start("arbiter")
        h.wait(lambda: h.state is not None and h.command is not None)
        h.pump(0.4)
        assert h.state.task_state == HuatSystemState.TASK_IDLE
        assert not h.state.can_drive and h.command.pedal_ratio == 0 and h.command.brake_force > 0
    h.case("arbiter_restart_no_permission", arbiter_restart)

    def monitor_restart():
        h.drive()
        h.state_runs = h.path_runs = False
        h.kill("monitor")
        h.wait(h.braked, 0.5 + JITTER)  # 监控心跳缺失锁存 FAULT
        # 清除旧实例缓存；以新实例双 Absent 租约作为重启就绪确认。
        h.monitor_inputs = None
        h.stop = None
        h.stop_at = -math.inf
        restart_at = time.monotonic() - h.started
        h.start("monitor")
        h.wait(lambda: h.monitor_inputs is not None
               and h.monitor_inputs["checked_at_steady_sec"] >= restart_at
               and h.monitor_inputs["vehicle_freshness"] == "Absent"
               and h.monitor_inputs["planner_freshness"] == "Absent"
               and h.safe())
        h.pump(0.5, h.safe)
        h.state_runs = h.path_runs = True
        h.wait(lambda: h.stop is False)
        h.pump(0.4)
        assert h.state.stop_kind == HuatSystemState.STOPKIND_FAULT and h.command.pedal_ratio == 0
        h.event_until("reset", h.ready)
    h.case("monitor_restart_timeout_and_fault_latch", monitor_restart)

    def clock_pause():
        h.drive()
        h.clock_runs = False
        h.state_runs = False  # 路径、监控心跳仍持续；仅定位停发
        h.wait(h.safe, STATE_TIMEOUT + JITTER)
        h.pump(0.4, h.safe)
        h.clock_runs = h.state_runs = True
        h.wait(h.ready)
    h.case("paused_ros_clock_vehicle_timeout", clock_pause)

    def clock_back():
        h.drive()
        h.state_runs = h.path_runs = False
        h.sim -= 20
        h.wait(h.safe, JITTER)
        h.path_runs = True
        h.pump(0.4, h.safe)
        h.state_runs = True
        h.wait(h.ready)
    h.case("ros_clock_rollback_needs_both_sources", clock_back)


def interrupted(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


signal.signal(signal.SIGTERM, interrupted)
rclpy.init()
h = Harness()
rc = 1
try:
    run(h)
    h.save("PASS")
    rc = 0
except Exception as exc:
    traceback.print_exc()
    h.save("FAIL", str(exc))
finally:
    h.close()
    h.destroy_node()
    rclpy.shutdown()
sys.exit(rc)
