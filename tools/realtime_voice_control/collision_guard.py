"""Local collision filtering shared by touch and voice; no ROS dependencies."""
from dataclasses import dataclass
import math
import time
import uuid

DIRECTIONS = {"front": 0.0, "left": math.pi/2, "rear": math.pi, "right": -math.pi/2}
LABELS = {"front": "前方", "rear": "后方", "left": "左侧", "right": "右侧"}
RANK = {"CLEAR": 0, "WARNING": 1, "SLOW": 2, "STOP": 3, "UNKNOWN": 4}
RANGE_EPS = 1e-6  # LaserScan float32 rounding must not miss an exact boundary.


@dataclass
class CollisionConfig:
    enabled: bool = True
    warning: float = .45
    slowdown: float = .30
    stop: float = .25
    side_warning: float = .225
    side_slowdown: float = .15
    side_stop: float = .10
    hysteresis: float = .03
    scan_timeout: float = .5
    linear_limit: float = .15
    angular_limit: float = .15
    front_degrees: float = 0.0

    def __post_init__(self):
        values = (self.warning, self.slowdown, self.stop,
                  self.side_warning, self.side_slowdown, self.side_stop, self.hysteresis,
                  self.scan_timeout, self.linear_limit, self.angular_limit, self.front_degrees)
        if not all(math.isfinite(x) for x in values):
            raise ValueError("防撞参数必须为有限数值")
        for name, warning, slowdown, stop in (
                ("前后", self.warning, self.slowdown, self.stop),
                ("左右", self.side_warning, self.side_slowdown, self.side_stop)):
            if not 0 < stop < slowdown < warning:
                raise ValueError(name+"防撞距离必须满足 0 < 停车 < 限速 < 预警")
            if not 0 <= self.hysteresis < min(slowdown-stop, warning-slowdown):
                raise ValueError(name+"防撞回差必须小于相邻距离阈值的差值")
        if not .1 <= self.scan_timeout <= 2:
            raise ValueError("雷达超时必须在0.1到2秒之间")
        if not 0 < self.linear_limit <= 1 or not 0 < self.angular_limit <= 1:
            raise ValueError("防撞限速输入必须在(0,1]内")

    def thresholds_for(self, zone):
        if zone in ("left", "right"):
            return self.side_warning, self.side_slowdown, self.side_stop
        return self.warning, self.slowdown, self.stop


class CollisionGuard:
    def __init__(self, config=None, clock=time.monotonic):
        self.c = config or CollisionConfig()
        self.clock = clock
        self.instance_id = uuid.uuid4().hex
        self.scan_at = None
        self.frame_id = ""
        self.distances = dict.fromkeys(DIRECTIONS)
        self.last_distances = dict.fromkeys(DIRECTIONS)
        self.last_distance_at = dict.fromkeys(DIRECTIONS)
        self.scan_times = []
        self.levels = dict.fromkeys(DIRECTIONS, "UNKNOWN")
        self.valid_levels = self.levels.copy()
        self.status = "NO_DATA" if self.c.enabled else "DISABLED"
        self.message = "等待有效雷达数据，暂不允许移动" if self.c.enabled else "防撞保护已关闭"
        self.action = "IDLE"
        self.required = []
        self.awaiting_command = False
        self.stop_directions = []
        self.stop_reason = ""
        self.last_signature = None
        self.last_event = None
        self.event_id = 0
        self.stop_episode = 0

    def update(self, msg):
        distances = dict.fromkeys(DIRECTIONS)
        try:
            low, high = float(msg.range_min), float(msg.range_max)
            start, step = float(msg.angle_min), float(msg.angle_increment)
            if not (all(math.isfinite(x) for x in (low, high, start, step))
                    and 0 <= low < high and high >= max(self.c.warning, self.c.side_warning)
                    and step != 0 and 1 <= len(msg.ranges) <= 20000):
                raise ValueError("invalid scan geometry")
            yaw = math.radians(self.c.front_degrees)
            for i, value in enumerate(msg.ranges):
                value = float(value)
                # No-return/invalid rays do not prove that a blind sector is clear.
                if not math.isfinite(value) or value <= 0 or not low <= value <= high:
                    continue
                angle = start + i*step - yaw
                for zone, centre in DIRECTIONS.items():
                    delta = math.atan2(math.sin(angle-centre), math.cos(angle-centre))
                    if abs(delta) <= math.pi/4 + 1e-9:
                        old = distances[zone]
                        distances[zone] = value if old is None else min(value, old)
        except (ValueError, TypeError, AttributeError, OverflowError):
            distances = dict.fromkeys(DIRECTIONS)
        self.scan_at = self.clock()
        if any(value is not None for value in distances.values()):
            self.scan_times.append(self.scan_at)
            self.scan_times=self.scan_times[-40:]
        for zone,value in distances.items():
            if value is not None:
                self.last_distances[zone]=value
                self.last_distance_at[zone]=self.scan_at
        self.frame_id = str(getattr(getattr(msg, "header", None), "frame_id", ""))
        self.distances = distances
        for zone, distance in distances.items():
            old = self.valid_levels[zone]
            warning, slowdown, stop = self.c.thresholds_for(zone)
            if distance is None:
                level = "UNKNOWN"
            elif distance <= stop+RANGE_EPS or (old == "STOP" and distance <= stop+self.c.hysteresis+RANGE_EPS):
                level = "STOP"
            elif distance <= slowdown+RANGE_EPS or (old == "SLOW" and distance <= slowdown+self.c.hysteresis+RANGE_EPS):
                level = "SLOW"
            elif distance <= warning+RANGE_EPS or (old == "WARNING" and distance <= warning+self.c.hysteresis+RANGE_EPS):
                level = "WARNING"
            else:
                level = "CLEAR"
            self.levels[zone] = level
            if level != "UNKNOWN":
                self.valid_levels[zone] = level

    def fresh(self):
        return self.scan_at is not None and 0 <= self.clock()-self.scan_at <= self.c.scan_timeout

    def _set_display(self, status, message, zones):
        signature = (status, tuple(sorted(zones)), self.stop_episode)
        self.status, self.message = status, message
        # Keep the last alarm across clear scans, so polling phones cannot miss it.
        if status not in ("CLEAR", "DISABLED", "WAIT_COMMAND") and signature != self.last_signature:
            self.event_id += 1
            self.last_event = dict(id=self.event_id, status=status, message=message,
                                   directions=list(zones), at=self.clock())
        self.last_signature = signature

    def _blocked_reason(self, zones, fresh):
        if not fresh:
            return "雷达扫描过期" if self.scan_at is not None else "尚未收到雷达扫描"
        missing = [LABELS[z] for z in zones if self.levels[z] == "UNKNOWN"]
        close = {}
        for zone in zones:
            if self.levels[zone] == "STOP":
                stop = self.c.thresholds_for(zone)[2]
                close.setdefault(stop, []).append(LABELS[zone])
        reasons = []
        if missing:
            reasons.append("、".join(missing)+"无有效测距")
        for stop, names in close.items():
            reasons.append("、".join(names)+f"处于{stop*100:g}cm停车区（含释放回差）")
        return "；".join(reasons)

    def evaluate(self, linear, angular):
        """Return safe normalized inputs, a persistent scaling factor and a stop flag."""
        if not self.c.enabled:
            self.action = "PASS"
            self._set_display("DISABLED", "防撞保护已关闭", [])
            return linear, angular, 1.0, False
        required = []
        if linear > 1e-9:
            required.append("front")
        elif linear < -1e-9:
            required.append("rear")
        if abs(angular) > 1e-9:
            # Rotation sweeps the body around all sides, not only the turn side.
            required = list(DIRECTIONS)
        self.required = required
        fresh = self.fresh()
        blocked = [z for z in required if not fresh or self.levels[z] in ("STOP", "UNKNOWN")]
        if blocked:
            unknown = not fresh or any(self.levels[z] == "UNKNOWN" for z in blocked)
            reason = self._blocked_reason(blocked, fresh)
            if not self.awaiting_command or blocked != self.stop_directions:
                self.stop_episode += 1
            self.awaiting_command = True
            self.stop_directions, self.stop_reason = blocked, reason
            self.action = "STOP"
            self._set_display("NO_DATA" if unknown else "STOP",
                              reason+"：已停车并清除旧指令；请检查安全方向后重新操作", blocked)
            return 0.0, 0.0, 0.0, True
        if required:
            self.awaiting_command = False
            slow = [z for z in required if self.levels[z] == "SLOW"]
            if slow:
                ratios = [1.0]
                if abs(linear) > 1e-9:
                    ratios.append(self.c.linear_limit/abs(linear))
                if abs(angular) > 1e-9:
                    ratios.append(self.c.angular_limit/abs(angular))
                ratio = min(ratios)
                self.action = "LIMIT"
                self._set_display("SLOW", "、".join(LABELS[z] for z in slow)+"接近障碍：已限制运动幅度", slow)
                return linear*ratio, angular*ratio, ratio, False
            warning = [z for z in required if self.levels[z] == "WARNING"]
            if warning:
                self.action = "PASS"
                self._set_display("WARNING", "、".join(LABELS[z] for z in warning)+"接近障碍，请注意距离", warning)
                return linear, angular, 1.0, False
        self.action = "PASS" if required else "IDLE"
        if self.awaiting_command:
            still_blocked = [z for z in self.stop_directions if not fresh or self.levels[z] in ("STOP", "UNKNOWN")]
            if still_blocked:
                status = "NO_DATA" if not fresh or any(self.levels[z]=="UNKNOWN" for z in still_blocked) else "STOP"
                self._set_display(status, self._blocked_reason(still_blocked, fresh)+"：保持停车，可向已确认安全的方向重新发令", still_blocked)
            else:
                self._set_display("WAIT_COMMAND", "停车条件已解除，等待新指令；不会自动恢复旧动作", [])
        elif not fresh:
            self._set_display("NO_DATA", "等待有效雷达数据，暂不允许移动", [])
        else:
            known = [z for z in DIRECTIONS if self.levels[z] != "UNKNOWN"]
            highest = max((self.levels[z] for z in known), key=lambda v:RANK[v], default="UNKNOWN")
            zones = [z for z in known if self.levels[z] == highest]
            if highest in ("STOP", "SLOW", "WARNING"):
                self._set_display(highest, "、".join(LABELS[z] for z in zones)+"附近有障碍，仅允许向安全方向操作", zones)
            elif not known:
                self._set_display("NO_DATA", "雷达未提供有效测距，暂不允许移动", list(DIRECTIONS))
            elif len(known) < len(DIRECTIONS):
                self._set_display("PARTIAL", "部分方向无有效测距，该方向的运动会被拦截", [z for z in DIRECTIONS if z not in known])
            else:
                self._set_display("CLEAR", "防撞保护已启用，雷达数据正常", [])
        return linear, angular, 1.0, False

    def snapshot(self):
        return dict(enabled=self.c.enabled, status=self.status, message=self.message, action=self.action,
                    instance_id=self.instance_id, last_event=self.last_event,
                    awaiting_new_command=self.awaiting_command, frame_id=self.frame_id,
                    fresh=self.fresh(), scan_age_ms=round((self.clock()-self.scan_at)*1000,1) if self.scan_at is not None else None,
                    distances=self.distances.copy(), levels=self.levels.copy(), required=list(self.required),
                    last_distances=self.last_distances.copy(),
                    distance_age_ms={zone:round((self.clock()-at)*1000,1) if at is not None else None for zone,at in self.last_distance_at.items()},
                    scan_hz=round((len(self.scan_times)-1)/(self.scan_times[-1]-self.scan_times[0]),2) if len(self.scan_times)>1 and self.scan_times[-1]>self.scan_times[0] else 0.,
                    thresholds=dict(warning=self.c.warning,slowdown=self.c.slowdown,stop=self.c.stop,hysteresis=self.c.hysteresis),
                    thresholds_by_direction={zone:dict(zip(("warning", "slowdown", "stop"), self.c.thresholds_for(zone))) for zone in DIRECTIONS},
                    limits=dict(linear=self.c.linear_limit,angular=self.c.angular_limit),
                    reference="雷达原点距离", front_degrees=self.c.front_degrees)
