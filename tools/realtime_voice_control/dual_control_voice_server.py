#!/usr/bin/env python3
"""Two phone controllers and streaming Chinese speech, with one ROS 2 publisher.

Run on the ROS 2 Linux host; see README.md for HTTPS and phone setup.
Core logic imports without ROS, aiohttp, or Vosk so it can be tested separately.
"""
import argparse
import asyncio
from collections import deque
from dataclasses import dataclass
import json
import logging
import math
import os
from pathlib import Path
import queue
import re
import secrets
import ssl
import struct
import threading
import time
from collision_guard import CollisionConfig, CollisionGuard
from manual_takeover import ManualTakeover
from scan_decoder import decode_scan

LOG = logging.getLogger("dual_voice")
VERSION = '2026.10.30-final.2'
ROOT = Path(__file__).resolve().parent
SAMPLE_RATE = 16000
PCM_BYTES = 640  # 20 ms, mono, signed 16 bit little endian
HEADER = struct.Struct("<dI")  # echoed server tick, frame sequence


@dataclass
class Config:
    cmd_topic: str = "cmd_vel_bl"
    imu_topic: str = "imu"
    hz: float = 50.0
    max_linear: float = 20.0
    max_angular: float = 300.0
    voice_linear: float = 0.35
    voice_angular: float = 0.50
    hold_seconds: float = 0.0  # 0: retain each slot's input while its audio is live
    heartbeat_timeout: float = 1.0
    audio_timeout: float = 0.5
    max_audio_age: float = 0.35
    fast_partial: bool = True
    stable_seconds: float = 0.08
    imu_safety: bool = False
    collision_enabled: bool = True
    scan_topic: str = "/scan"
    collision_warning: float = .45
    collision_slowdown: float = .30
    collision_stop: float = .25
    collision_side_warning: float = .225
    collision_side_slowdown: float = .15
    collision_side_stop: float = .10
    scan_timeout: float = .5
    scan_front_degrees: float = 0.0


ALIASES = {
    "forward": ("前进", "向前", "向前走", "往前走"),
    "backward": ("后退", "向后", "向后走", "往后退"),
    "left": ("左转", "向左", "向左转", "往左转"),
    "right": ("右转", "向右", "向右转", "往右转"),
    "faster": ("加速", "快一点", "快点", "提高速度"),
    "slower": ("减速", "慢一点", "慢点", "降低速度"),
    "stop": ("停", "停止", "停车", "停下", "停下来"),
    "estop": ("急停", "紧急停止", "紧急停车"),
}


def command_grammar(model):
    """Use words present in this model, including rejection phrases and unknowns."""
    def tokenize(phrase):
        if not phrase:
            return []
        for size in range(len(phrase), 0, -1):
            word = phrase[:size]
            if model.vosk_model_find_word(word) >= 0:
                rest = tokenize(phrase[size:])
                if rest is not None:
                    return [word] + rest
        return None

    phrases = set()
    for aliases in ALIASES.values():
        for text in aliases:
            for prefix in ("", "请", "小车", "不要", "别", "不能", "不许", "取消"):
                words = tokenize(prefix + text)
                if words:
                    phrases.add(" ".join(words))
    for text in ("解除急停", "恢复控制"):
        words = tokenize(text)
        if words:
            phrases.add(" ".join(words))
    phrases.add("[unk]")
    return json.dumps(sorted(phrases), ensure_ascii=False)


def parse_commands(text):
    """Segment only complete fixed commands, never arbitrary sentence keywords."""
    text = re.sub(r"[\s，。！？、,.!?：:;；]", "", text)
    if any(word in text for word in ("解除", "恢复", "不要", "别", "不能", "不许", "取消")):
        return []
    phrases = sorted(((prefix+alias, intent) for intent, aliases in ALIASES.items()
                      for alias in aliases for prefix in ("", "小车", "请")), reverse=True,
                     key=lambda item: len(item[0]))
    memo = {}
    def segment(pos):
        if pos == len(text):
            return []
        if pos not in memo:
            memo[pos] = None
            for phrase, intent in phrases:
                if text.startswith(phrase, pos):
                    tail = segment(pos+len(phrase))
                    if tail is not None:
                        memo[pos] = [intent] + tail
                        break
        return memo[pos]
    # Recognition output is short; cap work on arbitrary input.
    return (segment(0) or []) if len(text) <= 160 else []


def parse_intent(text):
    commands = parse_commands(text)
    return commands[0] if len(commands) == 1 else None


class IntentGate:
    """Deduplicate one utterance; optionally accept stable provisional commands."""
    def __init__(self, config):
        self.config = config
        self.candidate = None
        self.since = 0.0
        self.fired = None
        self.committed = []

    def feed(self, text, final, now):
        commands = parse_commands(text)
        if not commands and "[unk]" in text:
            recovered = parse_commands(text.replace("[unk]", ""))
            # Unknown acoustic material may not hide a stop; it never permits
            # a direction or a speed increase to be inferred through noise.
            if "stop" in recovered or "estop" in recovered:
                commands = recovered
        intent = "estop" if "estop" in commands else "stop" if "stop" in commands else commands[-1] if commands else None
        result = None
        prefix_ok = commands[:len(self.committed)] == self.committed
        new_command = bool(commands) and prefix_ok and len(commands) > len(self.committed)
        urgent = intent in ("stop", "estop")
        if final:
            if new_command or (urgent and commands != self.committed):
                result = intent
            self.candidate = self.fired = None
            self.committed = []
            return result
        candidate = tuple(commands)
        if candidate != self.candidate:
            self.candidate, self.since = candidate, now
        stable = self.config.fast_partial and now - self.since >= self.config.stable_seconds
        if (new_command or (urgent and commands != self.committed)) and (urgent or stable):
            self.fired = result = intent
            self.committed = commands
        return result


class ControlState:
    def __init__(self, config, clock=time.monotonic):
        self.c = config
        self.clock = clock
        self.lock = threading.RLock()
        self.clients = {}
        self.slots = dict.fromkeys(("FULL", "LINEAR", "ANGULAR", "LEFT", "RIGHT"))
        self.serial = 0
        self.command_revision = 0
        self.estop = False
        self.pitch = None
        self.imu_at = None
        self.safety = "NORMAL"
        self.recover_since = None
        self.last_cmd = (0.0, 0.0)
        self.message = "就绪"
        self.output_mode = "STARTING"
        self.publisher_state = "STARTING"
        self.agent_online = None
        self.agent_disconnects = 0
        self.publish_times = deque(maxlen=100)
        self.odom = None
        self.odom_at = None
        self.front_distance = None
        self.scan_at = None
        self.scan_errors = 0
        self.scan_error = ''
        self.takeover = ManualTakeover(clock)
        self.escape_mode_owner = None
        self.collision = CollisionGuard(CollisionConfig(enabled=config.collision_enabled,
            warning=config.collision_warning,slowdown=config.collision_slowdown,stop=config.collision_stop,
            side_warning=config.collision_side_warning,side_slowdown=config.collision_side_slowdown,side_stop=config.collision_side_stop,
            scan_timeout=config.scan_timeout,front_degrees=config.scan_front_degrees),clock)

    def _invalidate(self, cid=None):
        self.command_revision+=1
        targets = self.clients.values() if cid is None else (self.clients[cid],)
        for client in targets:
            self.serial += 1
            client["epoch"] = self.serial
            client["accept_after"] = self.clock()

    def _zero(self, cid=None):
        targets = self.clients.values() if cid is None else (self.clients[cid],)
        for client in targets:
            client["value"] = 0.0
            client["linear"] = client["angular"] = 0.0
            client["source"] = "idle"
            client["voice_until"] = None
            client["accepted_at"] = None
        self._invalidate(cid)

    def _connected(self, cid):
        if cid not in self.clients:
            raise ValueError("请先连接控制服务")
        return self.clients[cid]

    def _require(self, cid):
        client = self._connected(cid)
        if client["slot"] is None:
            raise ValueError("已连接，请先选择并分配控制角色")
        return client

    def connect(self, cid):
        with self.lock:
            if not isinstance(cid, str) or not 1 <= len(cid) <= 100:
                raise ValueError("client_id 无效")
            if cid in self.clients:
                self.clients[cid]["heartbeat"] = self.clock()
                return
            if len(self.clients) >= 2:
                raise ValueError("BUSY: 已有两个终端连接")
            self.serial += 1
            self.clients[cid] = dict(slot=None, value=0.0, linear=0.0, angular=0.0, heartbeat=self.clock(),
                                     mic=None, audio_at=None, speech="", speech_kind="",
                                     source="idle", voice_until=None, epoch=self.serial,
                                     accept_after=self.clock(), accepted_at=None, reported=False,
                                     audio_status="关闭", audio_recoveries=0, dropped_frames=0,
                                     decode_ms=0.0, queue_ms=0.0, peak_decode_ms=0.0,
                                     voice_linear_gain=self.c.voice_linear, voice_angular_gain=self.c.voice_angular,
                                     voice_feedback="等待口令", last_intent=None)
            self.clients[cid]['touch_until'] = 0.0
            self.message = "终端已连接，请分配控制角色"

    def claim(self, cid, role):
        with self.lock:
            client = self._connected(cid)
            if role not in self.slots:
                raise ValueError("角色应为 FULL、LINEAR、ANGULAR、LEFT 或 RIGHT")
            owner = self.slots[role]
            if owner is not None and owner != cid:
                raise ValueError(role + " 已被另一终端占用")
            peer_roles = {item["slot"] for key, item in self.clients.items() if key != cid and item["slot"]}
            allowed = {"FULL": set(), "LINEAR": {"ANGULAR"}, "ANGULAR": {"LINEAR"},
                       "LEFT": {"RIGHT"}, "RIGHT": {"LEFT"}}[role]
            if not peer_roles.issubset(allowed):
                raise ValueError("角色冲突：FULL 独占；双端请选择 LINEAR/ANGULAR 或 LEFT/RIGHT")
            if client["slot"] == role:
                return
            self._release_escape_mode(cid)
            if self.takeover.cancel(cid):
                self._zero()
            self.end_voice(cid)
            self._zero(cid)
            if client["slot"] is not None:
                self.slots[client["slot"]] = None
            self.slots[role] = cid
            client["slot"] = role
            client["voice_linear_gain"], client["voice_angular_gain"] = self.c.voice_linear, self.c.voice_angular
            self.message = role + " 控制权已分配"

    def release_role(self, cid):
        with self.lock:
            client = self._connected(cid)
            self._release_escape_mode(cid)
            if self.takeover.cancel(cid):
                self._zero()
            self.end_voice(cid)
            self._zero(cid)
            if client["slot"] is not None:
                self.slots[client["slot"]] = None
            client["slot"] = None
            self.message = "已释放角色，连接保留"

    def join(self, cid, role=None):
        """Compatibility with the original LEFT/RIGHT join endpoint."""
        with self.lock:
            self.connect(cid)
            if role is None:
                role = self.clients[cid]["slot"] or next((key for key in ("LEFT", "RIGHT") if self.slots[key] is None), None)
            if role is None:
                raise ValueError("BUSY: 已有两个控制端在线")
            self.claim(cid, role)

    def leave(self, cid):
        with self.lock:
            if cid not in self.clients:
                return
            self._release_escape_mode(cid)
            if self.takeover.cancel(cid):
                self._zero()
            if self.clients[cid]["slot"] is not None:
                self.slots[self.clients[cid]["slot"]] = None
            del self.clients[cid]

    def heartbeat(self, cid):
        with self.lock:
            if cid in self.clients:
                self.clients[cid]["heartbeat"] = self.clock()

    def stop(self, cid=None, latch=False):
        with self.lock:
            if cid is not None:
                self._connected(cid) if latch else self._require(cid)
            if self.takeover.cancel():
                self._zero()
                for client in self.clients.values():client['touch_until']=self.clock()+.6
            self._zero(None if latch or cid is None else cid)
            self.estop = self.estop or latch
            self.message = "急停已锁定，须手动复位" if self.estop else "本端已归零"

    def reset(self, cid):
        with self.lock:
            self._require(cid)
            self.takeover.cancel()
            self._zero()
            self.estop = False
            for client in self.clients.values():
                client["voice_linear_gain"], client["voice_angular_gain"] = self.c.voice_linear, self.c.voice_angular
            self.message = "急停已人工复位，请重新发出命令"

    def _blocked(self):
        return self.estop or (self.output_mode=='ROS2' and self.agent_online is False) or (self.c.imu_safety and self.safety not in ("NORMAL", "WARNING"))

    def _release_escape_mode(self, cid):
        if self.escape_mode_owner == cid:
            self.escape_mode_owner = None
            self._zero()
            self.message = '脱困发起端已离线或释放角色，已停车并恢复雷达联锁'

    def set_escape_mode(self, cid, enabled, epoch):
        with self.lock:
            client=self._require(cid)
            if not isinstance(enabled,bool):
                raise ValueError('enabled 必须为布尔值')
            if isinstance(epoch,bool) or not isinstance(epoch,int) or epoch!=client['epoch']:
                raise ValueError('旧模式切换请求已失效，请重新操作')
            if enabled == (self.escape_mode_owner is not None):
                return
            client['heartbeat']=self.clock()
            self.takeover.cancel()
            self._zero()
            self.escape_mode_owner=cid if enabled else None
            for item in self.clients.values():item['touch_until']=0.
            self.message='人工脱困已开启：雷达仅监测，语音和按键均由人控制，请重新发令' if enabled else '已恢复雷达联锁并停车，请重新发令'
            LOG.info('escape_mode enabled=%s owner=%s',enabled,cid)

    def update_agent_presence(self, present):
        with self.lock:
            if self.agent_online is True and present is False:
                self.agent_disconnects+=1
                self.stop(latch=True)
                self.message='Agent 通信中断，已急停；恢复后须人工复位并重新发令'
            self.agent_online=present

    def command(self, cid, value, expected_epoch=None):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("value 必须为有限数值")
        with self.lock:
            client = self._require(cid)
            if expected_epoch is not None and expected_epoch != client["epoch"]:
                raise ValueError("旧触控请求已失效，请重新操作")
            if self._blocked():
                raise ValueError("当前急停或姿态保护阻止运动")
            if self.takeover.current:
                raise ValueError('人工脱困正在进行，普通指令暂不执行；可随时停止或急停')
            # Invalidate only this phone's queued speech; preserve the peer slot.
            self._invalidate(cid)
            client["value"] = max(-1.0, min(1.0, value))
            client["linear"] = client["value"] if client["slot"] in ("FULL", "LINEAR") else 0.0
            client["angular"] = client["value"] if client["slot"] == "ANGULAR" else 0.0
            client["source"], client["voice_until"] = "manual", None
            client["accepted_at"] = None
            client["heartbeat"] = self.clock()
            client['touch_until'] = self.clock()+.6
            self.message = "手动控制"
            self.output()
            LOG.info('touch cid=%s input=(%.2f,%.2f) output=%s collision=%s',cid,client['linear'],client['angular'],self.last_cmd,self.collision.action)

    def axis_command(self, cid, linear, angular, expected_epoch=None):
        for value in (linear, angular):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("两轴输入必须为有限数值")
        with self.lock:
            client = self._require(cid)
            role = client["slot"]
            if role in ("LEFT", "RIGHT"):
                raise ValueError("原左右协同角色使用单端输入")
            if role == "LINEAR" and angular != 0:
                raise ValueError("LINEAR 角色只能控制前后")
            if role == "ANGULAR" and linear != 0:
                raise ValueError("ANGULAR 角色只能控制转向")
            if expected_epoch is not None and expected_epoch != client["epoch"]:
                raise ValueError("旧触控请求已失效，请重新操作")
            if self._blocked():
                raise ValueError("当前急停或姿态保护阻止运动")
            if self.takeover.current:
                raise ValueError('人工脱困正在进行，普通指令暂不执行；可随时停止或急停')
            self._invalidate(cid)
            client["linear"] = max(-1.0, min(1.0, linear))
            client["angular"] = max(-1.0, min(1.0, angular))
            client["value"] = client["angular"] if role == "ANGULAR" else client["linear"]
            client["source"], client["voice_until"], client["accepted_at"] = "manual", None, None
            client["heartbeat"] = self.clock()
            client['touch_until'] = self.clock()+.6
            self.message = role + " 手动输入已更新"
            self.output()
            LOG.info('touch cid=%s input=(%.2f,%.2f) output=%s collision=%s',cid,linear,angular,self.last_cmd,self.collision.action)

    def attach_mic(self, cid, token):
        with self.lock:
            client = self._require(cid)
            if self.takeover.current:
                raise ValueError('人工脱困中，请结束后再开启语音')
            if client["mic"] is not None:
                raise ValueError("本终端已开启麦克风")
            client["mic"] = token
            client["audio_at"] = None
            client["audio_status"] = "初始化识别器"

    def detach_mic(self, cid, token, reason):
        with self.lock:
            client = self.clients.get(cid)
            if client and client["mic"] == token:
                client["mic"] = None
                client["audio_at"] = None
                client["audio_status"] = "关闭"
                if client["source"] == "voice":
                    self._zero(cid)
                self.message = reason

    def audio_received(self, cid, token):
        with self.lock:
            client = self._require(cid)
            if client["mic"] != token:
                raise ValueError("麦克风会话已结束")
            client["audio_at"] = self.clock()
            client["audio_status"] = "实时收音"

    def audio_recovery(self, cid, token, reason, dropped=0):
        with self.lock:
            client = self.clients.get(cid)
            if not client or client["mic"] != token:
                return
            if client["source"] == "voice":
                self._zero(cid)
            else:
                self._invalidate(cid)
            client["audio_recoveries"] += 1
            client["dropped_frames"] += dropped
            client["audio_status"] = "追赶实时输入"
            self.message = reason

    def audio_metrics(self, cid, token, queue_ms, decode_ms):
        with self.lock:
            client = self.clients.get(cid)
            if client and client["mic"] == token:
                client["queue_ms"], client["decode_ms"] = queue_ms, decode_ms
                client["peak_decode_ms"] = max(client["peak_decode_ms"], decode_ms)

    def mic_active(self, cid, token):
        with self.lock:
            client = self.clients.get(cid)
            return client is not None and client["mic"] == token

    def end_voice(self, cid):
        with self.lock:
            client = self._connected(cid)
            token = client["mic"]
            if token is not None:
                self.detach_mic(cid, token, "本端麦克风已关闭")

    def generation(self, cid):
        with self.lock:
            client = self._require(cid)
            return client["epoch"], client["accept_after"]

    def _change_speed(self, client, faster):
        role = client["slot"]
        keys = ("value",) if role in ("LEFT", "RIGHT") else tuple(
            key for key in ("linear", "angular")
            if role == "FULL" or role == {"linear":"LINEAR", "angular":"ANGULAR"}[key])
        active = changed = False
        for key in keys:
            value = client[key]
            if abs(value) < 1e-6:
                continue
            active = True
            base = self.c.voice_angular if key == "angular" else self.c.voice_linear
            floor, ceiling = min(.15, base), max(.65, base)
            magnitude = abs(value)
            target = max(magnitude, min(ceiling, round(magnitude+.10, 2))) if faster else min(magnitude, max(floor, round(magnitude-.10, 2)))
            if abs(target-magnitude) < 1e-6:
                continue
            client[key] = math.copysign(target, value)
            client["voice_"+("angular" if key == "angular" else "linear")+"_gain"] = target
            changed = True
        if not active:
            client["voice_feedback"] = "本端当前静止，请先发出方向口令；加减速不会自动起步"
            return False
        if not changed:
            client["voice_feedback"] = "本端已到当前方向的档位上限" if faster else "本端已到当前方向的档位下限；停车请说停止"
            return False
        if role not in ("LEFT", "RIGHT"):
            client["value"] = client["angular"] if role == "ANGULAR" else client["linear"]
        return True

    def speech_result(self, cid, token, text, final, intent, captured_at, epoch):
        with self.lock:
            client = self.clients.get(cid)
            now = self.clock()
            if not client or client["mic"] != token or epoch != client["epoch"]:
                return False
            if captured_at < client["accept_after"] or now - captured_at > self.c.max_audio_age:
                return False
            client["speech"] = text
            client["speech_kind"] = "final" if final else "partial"
            if intent in ("stop", "estop"):
                self.stop(cid, latch=(intent == "estop"))
                client["last_intent"] = intent
                client["voice_feedback"] = "已执行：整车急停锁定" if intent == "estop" else "已执行：本端停止"
                client["accepted_at"], client["reported"] = now, False
                LOG.info("accepted cid=%s intent=%s final=%s audio_age_ms=%.1f",cid,intent,final,(now-captured_at)*1000)
                return True
            if intent is None:
                if text and not parse_commands(text):
                    client["voice_feedback"] = "文本未匹配完整短口令，未执行运动"
                return False
            if intent not in ALIASES:
                return False
            if self.takeover.current or now < client.get('touch_until',0):
                client['voice_feedback'] = '触控优先：当前语音运动指令未执行；停止和急停仍优先'
                return False
            if self._blocked():
                client["voice_feedback"] = "未执行：急停或姿态保护，请检查状态并人工复位"
                return False
            role = client["slot"]
            if role is None:
                return False
            if intent in ("faster", "slower"):
                if not self._change_speed(client, intent == "faster"):
                    return False
                value = client["value"]
            elif role in ("FULL", "LINEAR", "ANGULAR"):
                if role == "LINEAR" and intent in ("left", "right"):
                    self.message = "当前是前后端，转向由 ANGULAR 端控制"
                    client["voice_feedback"] = "未执行：本端只有前后控制权"
                    return False
                if role == "ANGULAR" and intent in ("forward", "backward"):
                    self.message = "当前是转向端，前后由 LINEAR 端控制"
                    client["voice_feedback"] = "未执行：本端只有转向控制权"
                    return False
                client["linear"] = client["voice_linear_gain"] * {"forward": 1, "backward": -1}.get(intent, 0)
                client["angular"] = client["voice_angular_gain"] * {"left": 1, "right": -1}.get(intent, 0)
                value = client["angular"] if role == "ANGULAR" else client["linear"]
            elif intent in ("left", "right"):
                peers = [item for key, item in self.clients.items() if key != cid and item["slot"] in ("LEFT", "RIGHT")]
                if not peers:
                    self.message = "原双端算法单端只支持前后，转向需两个终端加入"
                    return False
                peer = peers[0]
                # Change only this slot. Place its value above/below the peer
                # so (RIGHT - LEFT) has the requested steering sign.
                sign = 1 if intent == "left" else -1
                side = -1 if client["slot"] == "LEFT" else 1
                value = peer["value"] + sign * side * client["voice_angular_gain"]
            else:
                value = client["voice_linear_gain"] * (1 if intent == "forward" else -1)
            client["value"] = max(-1.0, min(1.0, value))
            client["source"] = "voice"
            client["voice_until"] = now + self.c.hold_seconds if self.c.hold_seconds else None
            client["accepted_at"], client["reported"] = now, False
            client["last_intent"] = intent
            client["voice_feedback"] = "已执行：" + {"forward":"前进", "backward":"后退", "left":"左转", "right":"右转", "faster":"本端加一档", "slower":"本端减一档"}[intent]
            self.message = client["slot"] + " 语音输入: " + intent
            self.command_revision+=1
            self.output()
            if self.collision.action == "STOP":
                LOG.info("blocked cid=%s intent=%s collision=%s",cid,intent,self.collision.message)
                return False
            if self.collision.action == "LIMIT":
                client["voice_feedback"] += "（防撞限速）"
            LOG.info("accepted cid=%s intent=%s final=%s audio_age_ms=%.1f",
                     cid, intent, final, (now - captured_at) * 1000)
            return True

    def update_odom(self, linear, angular):
        with self.lock:
            if math.isfinite(linear) and math.isfinite(angular):
                self.odom, self.odom_at = (linear, angular), self.clock()

    def update_scan(self, msg):
        with self.lock:
            self.collision.update(msg)
            self.front_distance = self.collision.distances["front"]
            self.scan_at = self.collision.scan_at
            self.output()

    def update_scan_bytes(self, data):
        try:
            scan=decode_scan(data)
        except (ValueError,UnicodeError,struct.error) as error:
            with self.lock:
                self.scan_errors+=1;self.scan_error=str(error)
            self.update_scan(None)
            return
        self.update_scan(scan)

    def escape(self, cid, phase, token, direction=None, epoch=None):
        with self.lock:
            client=self._require(cid)
            if not isinstance(token,str) or not 8<=len(token)<=80:
                raise ValueError('脱困操作标识无效')
            if phase=='release':
                if self.takeover.cancel(cid,token):
                    self._zero()
                    for item in self.clients.values():item['touch_until']=self.clock()+.6
                    LOG.info('escape cid=%s released output=zero',cid)
                return
            if self._blocked():
                raise ValueError('急停或姿态保护仍然有效，禁止脱困')
            self.output()  # Expire a lease before processing a late renewal.
            if self._blocked():
                raise ValueError('急停或姿态保护仍然有效，禁止脱困')
            if phase=='start':
                if isinstance(epoch,bool) or not isinstance(epoch,int) or epoch!=client['epoch']:
                    raise ValueError('旧脱困请求已失效，请重新按住')
                self.takeover.begin(cid,token,client['slot'],direction)
                self._zero()
                LOG.info('escape cid=%s direction=%s started',cid,direction)
            elif phase=='keep':
                self.takeover.renew(cid,token)
            else:
                raise ValueError('脱困操作无效')
            client['heartbeat']=self.clock()
            self.message=self.takeover.message
            self.output()

    def update_pitch(self, pitch):
        with self.lock:
            now = self.clock()
            self.pitch, self.imu_at = pitch, now
            if not self.c.imu_safety:
                return
            if not math.isfinite(pitch):
                self.safety = "SAFETY"
                self.recover_since = None
                self._zero()
            elif abs(pitch) >= 12.0:
                self.safety = "SAFETY"
                self.recover_since = None
                self._zero()
            elif self.safety in ("SAFETY", "RECOVERY", "IMU_STALE"):
                if abs(pitch) <= 4.0:
                    if self.recover_since is None:
                        self.recover_since = now
                    self.safety = "RECOVERY"
                    if now - self.recover_since >= 1.0:
                        self.safety, self.recover_since = "NORMAL", None
                        self._zero()
                else:
                    self.safety, self.recover_since = "SAFETY", None
            else:
                self.safety = "WARNING" if abs(pitch) >= 8.0 else "NORMAL"

    def output(self):
        with self.lock:
            now = self.clock()
            for cid, client in list(self.clients.items()):
                if now - client["heartbeat"] > self.c.heartbeat_timeout:
                    self.leave(cid)
            active=self.takeover.current
            if active and (self.takeover.expired() or active['owner'] not in self.clients or
                           self.clients[active['owner']]['slot'] != active['role'] or self._blocked() or
                           (self.c.imu_safety and (self.imu_at is None or now-self.imu_at>.5))):
                self.takeover.cancel(reason='脱困已因松手、续期超时、权限变化或保护停止；请重新操作')
                LOG.info('escape stopped: lease, owner or safety changed')
                self._zero()
                for client in self.clients.values():client['touch_until']=now+.6
            if self.takeover.current:
                active=self.takeover.current
                self.collision.evaluate(0.,0.)
                self.last_cmd=(active['linear']*self.c.max_linear,active['angular']*self.c.max_angular)
                return self.last_cmd
            if self.c.imu_safety and (self.imu_at is None or now - self.imu_at > 0.5):
                if self.safety != "IMU_STALE":
                    self.safety, self.recover_since = "IMU_STALE", None
                    self._zero()
            for cid, client in self.clients.items():
                if client["source"] != "voice":
                    continue
                audio_at, until = client["audio_at"], client["voice_until"]
                if (until is not None and now >= until) or audio_at is None or now - audio_at > self.c.audio_timeout:
                    self._zero(cid)
                    self.message = client["slot"] + " 语音到期或音频中断，本端已归零"
            if self._blocked():
                linear = angular = 0.0
            elif self.slots["FULL"]:
                owner = self.clients[self.slots["FULL"]]
                linear, angular = owner["linear"], owner["angular"]
            elif self.slots["LINEAR"] or self.slots["ANGULAR"]:
                linear = self.clients[self.slots["LINEAR"]]["linear"] if self.slots["LINEAR"] else 0.0
                angular = self.clients[self.slots["ANGULAR"]]["angular"] if self.slots["ANGULAR"] else 0.0
            else:
                values = [self.clients[self.slots[key]]["value"] for key in ("LEFT", "RIGHT") if self.slots[key] in self.clients]
                if len(values) == 1:
                    left = right = values[0]
                elif len(values) == 2:
                    left, right = values
                else:
                    left = right = 0.0
                linear, angular = (left + right) / 2.0, (right - left) / 2.0
            if self.escape_mode_owner is not None:
                # Keep distances/diagnostics running, but never let radar gate
                # voice, touch, either phone or either axis in manual mode.
                self.collision.evaluate(0.,0.)
                factor,stopped=1.,False
            else:
                linear, angular, factor, stopped = self.collision.evaluate(linear, angular)
            if stopped:
                self._zero()
                for client in self.clients.values():
                    client["voice_feedback"] = "防撞已拦截：" + self.collision.message
            elif factor < 1.0-1e-9:
                for client in self.clients.values():
                    for key in ("value", "linear", "angular"):
                        client[key] *= factor
                self._invalidate()
            self.last_cmd = linear * self.c.max_linear, angular * self.c.max_angular
            return self.last_cmd

    def published(self, output):
        with self.lock:
            self.publish_times.append(self.clock())
            self.publisher_state = "RUNNING"
            if output != self.last_cmd:
                return
            for cid, client in self.clients.items():
                if client["accepted_at"] is not None and not client["reported"]:
                    LOG.info("cid=%s accept_to_publish_ms=%.1f", cid,
                             (self.clock() - client["accepted_at"]) * 1000)
                    client["reported"] = True

    def snapshot(self, cid):
        with self.lock:
            client = self.clients.get(cid, {})
            hz = (len(self.publish_times)-1)/(self.publish_times[-1]-self.publish_times[0]) if len(self.publish_times)>1 and self.publish_times[-1]>self.publish_times[0] else 0.0
            return dict(state_tick=self.clock(),connected=cid in self.clients, my_slot=client.get("slot"), slots=self.slots.copy(),
                        safety_state=self.safety, estop=self.estop,
                        pitch_deg=self.pitch if self.pitch is not None and math.isfinite(self.pitch) else None,
                        message=self.message, last_cmd=dict(zip(("linear_x", "angular_z"), self.last_cmd)),
                        clients={key:dict(slot=item["slot"],value=item["value"],linear=item["linear"],angular=item["angular"],source=item["source"])
                                 for key,item in self.clients.items()},
                        my_epoch=client.get("epoch"), hold_seconds=self.c.hold_seconds,
                        my_input=dict(value=client.get("value",0),linear=client.get("linear",0),angular=client.get("angular",0)),
                        voice_feedback=client.get("voice_feedback","等待连接并分配角色"), last_intent=client.get("last_intent"),
                        voice_settings=dict(linear=client.get("voice_linear_gain",self.c.voice_linear),angular=client.get("voice_angular_gain",self.c.voice_angular)),
                        collision=self.collision.snapshot(),
                        manual_takeover=self.takeover.snapshot(),
                        driving_mode=dict(escape=self.escape_mode_owner is not None,owner=self.escape_mode_owner,
                                          radar_interlock=self.collision.c.enabled and self.escape_mode_owner is None),
                        scan_diagnostics=dict(rejected_frames=self.scan_errors,last_error=self.scan_error),
                        feedback=dict(odom=self.odom if self.odom_at is not None and self.clock()-self.odom_at < 1 else None,
                                      front_distance_m=self.front_distance if self.scan_at is not None and self.clock()-self.scan_at < 1 else None),
                        runtime=dict(mode=self.output_mode,publisher_state=self.publisher_state,cmd_topic=self.c.cmd_topic,
                                     version=VERSION,
                                     agent_listener=self.agent_online,agent_disconnects=self.agent_disconnects,
                                     domain_id=os.environ.get("ROS_DOMAIN_ID","0"),publish_hz=round(hz,1),
                                     voice_linear=self.c.voice_linear,voice_angular=self.c.voice_angular,
                                     last_publish_age_ms=round((self.clock()-self.publish_times[-1])*1000,1) if self.publish_times else None),
                        audio={key:client.get(key,0) for key in ("audio_status","decode_ms","queue_ms","peak_decode_ms","audio_recoveries","dropped_frames")},
                        speech=client.get("speech", ""), speech_kind=client.get("speech_kind", ""),
                        mic_connected=client.get("mic") is not None,
                        fast_partial=self.c.fast_partial)


class AudioWorker:
    """One recognizer per phone; decoding never runs in the ROS or web loop."""
    def __init__(self, state, cid, token, recognizer_factory):
        self.state, self.cid, self.token = state, cid, token
        self.factory = recognizer_factory
        # Absorb short decoder bursts without buffering seconds of old speech.
        self.queue = queue.Queue(maxsize=12)
        self.closed = threading.Event()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self.run, name="asr-" + cid[:12], daemon=True)

    def put(self, pcm, captured_at):
        if self.closed.is_set():
            raise ValueError("识别线程已停止，请重新开启语音")
        try:
            self.queue.put_nowait((pcm, captured_at))
        except queue.Full:
            self.recover("识别短时积压，已丢弃旧音频并恢复实时收音")
            self.queue.put_nowait((pcm, captured_at))

    def recover(self, reason, dropped=0):
        while True:
            try:
                self.queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        self.state.audio_recovery(self.cid, self.token, reason, dropped)

    def close(self, reason="麦克风已关闭"):
        self.closed.set()
        self.state.detach_mic(self.cid, self.token, reason)

    def run(self):
        try:
            rec = self.factory()
            self.ready.set()
            gate = IntentGate(self.state.c)
            epoch, _ = self.state.generation(self.cid)
            while not self.closed.is_set():
                try:
                    pcm, stamp = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if not self.state.mic_active(self.cid, self.token):
                    break
                if self.state.clock() - stamp > self.state.c.max_audio_age:
                    self.recover("旧音频已跳过，等待新的实时口令", dropped=1)
                    continue
                current_epoch, after = self.state.generation(self.cid)
                if current_epoch != epoch:
                    rec.Reset()
                    gate = IntentGate(self.state.c)
                    epoch = current_epoch
                if stamp < after:
                    continue
                started = self.state.clock()
                final = rec.AcceptWaveform(pcm)
                result = json.loads(rec.Result() if final else rec.PartialResult())
                text = result.get("text" if final else "partial", "")
                self.state.audio_metrics(self.cid, self.token, (started-stamp)*1000,
                                         (self.state.clock()-started)*1000)
                intent = gate.feed(text, final, self.state.clock())
                if text or intent:
                    self.state.speech_result(self.cid, self.token, text, final, intent, stamp, epoch)
                if intent:
                    LOG.info("decode_ms=%.1f", (self.state.clock() - started) * 1000)
        except Exception as exc:
            LOG.exception("speech worker stopped: %s", exc)
            self.close(str(exc))


class StaleAudioError(ValueError):
    pass


class AudioProtocol:
    """Reject sequence gaps and audio queued across a delayed network connection."""
    def __init__(self, config, clock=time.monotonic):
        self.c, self.clock = config, clock
        self.ticks = deque(maxlen=8)
        self.sequence = 0

    def tick(self):
        stamp = self.clock()
        self.ticks.append(stamp)
        return stamp

    def decode(self, frame):
        if len(frame) != HEADER.size + PCM_BYTES:
            raise ValueError("音频帧必须为20ms的16kHz单声道PCM16")
        stamp, sequence = HEADER.unpack_from(frame)
        if sequence != self.sequence:
            raise ValueError("音频序号不连续，请重新开启语音")
        self.sequence = (self.sequence + 1) % (2 ** 32)
        if not math.isfinite(stamp) or stamp > self.clock():
            raise ValueError("无效音频时间戳")
        if self.clock() - stamp > self.c.max_audio_age:
            raise StaleAudioError("网络旧音频已跳过，保持实时收音")
        if stamp not in self.ticks:
            raise ValueError("音频时间戳必须来自服务端")
        return frame[HEADER.size:], stamp


class CommandCadence:
    """Check protection at 50Hz; changed values go now, steady values at 10Hz."""
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.previous=None;self.sent_at=None;self.revision=None

    def due(self,output,revision=None):
        now=self.clock()
        if output!=self.previous or revision!=self.revision or self.sent_at is None or now-self.sent_at>=.1-1e-9:
            self.previous=output;self.sent_at=now;self.revision=revision;return True
        return False


class PublisherThread:
    def __init__(self, state, dry_run=False):
        self.state, self.dry_run = state, dry_run
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, name="ros-publisher", daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(15):
            raise RuntimeError("ROS 2发布线程初始化超时")
        if self.error:
            raise RuntimeError(str(self.error))

    def run(self):
        node = context = executor = None
        try:
            self.state.output_mode = "DRY_RUN" if self.dry_run else "ROS2"
            if self.dry_run:
                self.ready.set()
                previous = None
                while not self.stop_event.is_set():
                    output = self.state.output()
                    if output != previous:
                        LOG.info("DRY RUN linear_x=%.3f angular_z=%.3f", *output)
                        previous = output
                    self.state.published(output)
                    self.stop_event.wait(1.0 / self.state.c.hz)
                return
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import Imu
            from sensor_msgs.msg import LaserScan
            from nav_msgs.msg import Odometry

            context = Context()
            rclpy.init(context=context)
            node = Node("dual_terminal_voice_control", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            publisher = node.create_publisher(Twist, self.state.c.cmd_topic, 1)

            def imu_callback(msg):
                q = msg.orientation
                norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
                if msg.orientation_covariance[0] < 0 or not math.isfinite(norm) or norm < 1e-8:
                    self.state.update_pitch(float("nan"))
                    return
                sinp = 2.0 * (q.w * q.y - q.z * q.x) / norm
                self.state.update_pitch(math.degrees(math.asin(max(-1.0, min(1.0, sinp)))))

            node.create_subscription(Imu, self.state.c.imu_topic, imu_callback, qos_profile_sensor_data)
            node.create_subscription(Odometry, "/odom_raw", lambda msg:self.state.update_odom(msg.twist.twist.linear.x,msg.twist.twist.angular.z), qos_profile_sensor_data)
            scan_qos = QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(LaserScan, self.state.c.scan_topic, self.state.update_scan_bytes, scan_qos,raw=True)

            last_agent_check=[0.]
            cadence=CommandCadence()
            def publish():
                if self.state.c.cmd_topic.strip('/')=='cmd_vel_bl' and time.monotonic()-last_agent_check[0]>.5:
                    last_agent_check[0]=time.monotonic()
                    try:
                        entries=Path('/proc/net/udp').read_text().splitlines()[1:]
                        present=any(row.split()[1].endswith(':22C3') for row in entries if len(row.split())>2)
                        self.state.update_agent_presence(present)
                    except OSError:pass
                with self.state.lock:
                    linear, angular = self.state.output()
                    revision=self.state.command_revision
                if not cadence.due((linear,angular),revision):
                    return
                msg = Twist()
                msg.linear.x, msg.angular.z = float(linear), float(angular)
                publisher.publish(msg)
                self.state.published((linear, angular))

            node.create_timer(1.0 / self.state.c.hz, publish)
            self.ready.set()
            while context.ok() and not self.stop_event.is_set():
                executor.spin_once(timeout_sec=0.05)
            publisher.publish(Twist())
        except Exception as exc:
            self.error = exc
            self.state.stop(latch=True)
            self.ready.set()
            LOG.exception("publisher stopped")
        finally:
            if executor:
                executor.shutdown(timeout_sec=1.0)
            if node:
                node.destroy_node()
            if context and context.ok():
                context.shutdown()

    def close(self):
        self.state.stop(latch=True)
        self.stop_event.set()
        self.thread.join(timeout=2.0)


def make_app(state, recognizer_factory=None, publisher=None):
    from aiohttp import web, WSMsgType
    app = web.Application(client_max_size=4096)
    workers, sockets = {}, set()

    async def index(request):
        return web.FileResponse(ROOT / "index.html",headers={'Cache-Control':'no-cache'})

    async def worklet(request):
        return web.Response(text=(ROOT / "pcm-worklet.js").read_text(encoding="utf-8"),
                            content_type="application/javascript")

    async def collision_script(request):
        return web.Response(text=(ROOT / "collision-alerts.js").read_text(encoding="utf-8"),
                            content_type="application/javascript",headers={"Cache-Control":"no-cache"})

    async def session_script(request):
        return web.Response(text=(ROOT / "control-session.js").read_text(encoding="utf-8"),
                            content_type="application/javascript",headers={"Cache-Control":"no-cache"})

    async def api(request):
        try:
            data = await request.json()
            if not isinstance(data, dict):
                raise ValueError("JSON必须是对象")
            cid = data.get("client_id", "")
            if not isinstance(cid, str):
                raise ValueError("client_id 无效")
            action = request.match_info["action"]
            if publisher and publisher.error:
                raise ValueError("ROS 2发布线程已退出，请重启服务")
            if action == "connect":
                state.connect(cid)
            elif action == "claim":
                state.claim(cid, data.get("role"))
            elif action == "release-role":
                state.release_role(cid)
            elif action == "join":
                state.join(cid, data.get("role"))
            elif action == "leave":
                state.leave(cid)
            elif action == "heartbeat":
                state.heartbeat(cid)
            elif action == "cmd":
                epoch = data.get("epoch")
                if isinstance(epoch, bool) or not isinstance(epoch, int):
                    raise ValueError("触控请求需要当前槽位epoch")
                if "linear" in data or "angular" in data:
                    state.axis_command(cid, data.get("linear", 0), data.get("angular", 0), epoch)
                else:
                    state.command(cid, data.get("value"), epoch)
            elif action == "voice-stop":
                state.end_voice(cid)
            elif action == "stop":
                state.stop(cid)
                state.clients[cid]['touch_until']=state.clock()+.6
            elif action == "escape":
                state.escape(cid,data.get('phase'),data.get('token'),data.get('direction'),data.get('epoch'))
            elif action == "escape-mode":
                state.set_escape_mode(cid,data.get('enabled'),data.get('epoch'))
            elif action == "estop":
                state.stop(cid, latch=True)
            elif action == "reset":
                state.reset(cid)
            else:
                raise web.HTTPNotFound()
            return web.json_response(dict(ok=True, **state.snapshot(cid)))
        except (ValueError, TypeError, KeyError) as exc:
            return web.json_response(dict(ok=False, error=str(exc)), status=400)

    async def audio(request):
        if recognizer_factory is None:
            return web.json_response(dict(error="服务未加载语音模型"), status=503)
        cid = request.query.get("client_id", "")
        token = secrets.token_hex(16)
        try:
            state.attach_mic(cid, token)
        except ValueError as exc:
            return web.json_response(dict(error=str(exc)), status=403)
        ws = web.WebSocketResponse(max_msg_size=1024, heartbeat=2.0)
        worker = AudioWorker(state, cid, token, recognizer_factory)
        workers[token] = worker
        sender = None
        try:
            await ws.prepare(request)
            sockets.add(ws)
            worker.thread.start()
            protocol = AudioProtocol(state.c, state.clock)

            async def send_ticks():
                while not ws.closed:
                    if worker.closed.is_set() or not state.mic_active(cid, token):
                        await ws.close(code=1008, message=b"speech worker stopped")
                        return
                    if not worker.ready.is_set():
                        await asyncio.sleep(0.02)
                        continue
                    await ws.send_json(dict(type="tick", stamp=protocol.tick()))
                    await asyncio.sleep(0.1)

            sender = asyncio.create_task(send_ticks())
            async for msg in ws:
                if msg.type == WSMsgType.BINARY:
                    try:
                        pcm, stamp = protocol.decode(msg.data)
                        state.audio_received(cid, token)
                        worker.put(pcm, stamp)
                    except StaleAudioError as exc:
                        worker.recover(str(exc), dropped=1)
                    except ValueError as exc:
                        worker.close(str(exc))
                        await ws.close(code=1008, message=str(exc).encode("utf-8")[:120])
                        break
                elif msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.ERROR):
                    break
                else:
                    await ws.close(code=1003, message=b"binary PCM only")
                    break
        finally:
            if sender:
                sender.cancel()
                await asyncio.gather(sender, return_exceptions=True)
            worker.close()
            workers.pop(token, None)
            sockets.discard(ws)
        return ws

    async def health(request):
        return web.json_response(dict(ok=not publisher or publisher.error is None,
                                      version=VERSION,topic=state.c.cmd_topic, voice=recognizer_factory is not None))

    async def status(request):
        state.output()
        return web.json_response(dict(ok=True, **state.snapshot(request.query.get("client_id", ""))))

    async def shutdown(application):
        state.stop(latch=True)
        for worker in list(workers.values()):
            worker.close()
        await asyncio.gather(*(ws.close(code=1001) for ws in list(sockets)))

    app.router.add_get("/", index)
    app.router.add_get("/pcm-worklet.js", worklet)
    app.router.add_get("/collision-alerts.js", collision_script)
    app.router.add_get("/control-session.js", session_script)
    app.router.add_get("/audio", audio)
    app.router.add_get("/health", health)
    app.router.add_get("/state", status)
    app.router.add_post("/{action}", api)
    app.on_shutdown.append(shutdown)
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, help="解压后的Vosk中文模型目录")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="仅打印输出，不连接ROS 2")
    parser.add_argument("--no-voice", action="store_true", help="仅运行双端触控，免Vosk")
    parser.add_argument("--confirmed-only", action="store_true", help="运动仅用确认结果；默认稳定中间结果触发")
    parser.add_argument("--free-vocabulary", action="store_true", help="使用通用词表，默认固定控制短口令词表")
    parser.add_argument("--hold-seconds", type=float, default=0.0, help="0为持续控制，或设置0.1到5秒的动作时限")
    parser.add_argument("--voice-linear", type=float, default=Config.voice_linear,
                        help="语音前后输入幅度，范围(0,1]，默认0.35")
    parser.add_argument("--voice-angular", type=float, default=Config.voice_angular,
                        help="语音转向输入幅度，范围(0,1]，默认0.40")
    parser.add_argument("--start-estopped", action="store_true", help="启动即锁定急停，分配角色后人工复位")
    parser.add_argument("--imu-safety", action="store_true")
    parser.add_argument("--cmd-topic", default="cmd_vel_bl")
    parser.add_argument("--imu-topic", default="imu")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--collision-warning",type=float,default=Config.collision_warning,help="前后预警距离，单位米")
    parser.add_argument("--collision-slowdown",type=float,default=Config.collision_slowdown,help="前后限速距离，单位米")
    parser.add_argument("--collision-stop",type=float,default=Config.collision_stop,help="前后停车距离，单位米")
    parser.add_argument("--collision-side-warning",type=float,default=Config.collision_side_warning,help="左右预警距离，单位米")
    parser.add_argument("--collision-side-slowdown",type=float,default=Config.collision_side_slowdown,help="左右限速距离，单位米")
    parser.add_argument("--collision-side-stop",type=float,default=Config.collision_side_stop,help="左右停车距离，单位米")
    parser.add_argument("--scan-timeout",type=float,default=.5)
    parser.add_argument("--scan-front-degrees",type=float,default=0.0,help="雷达坐标中车头方向的角度")
    parser.add_argument("--no-collision-guard",action="store_true",help="仅用于离线联调，显式关闭防撞保护")
    args = parser.parse_args()
    try:
        CollisionConfig(warning=args.collision_warning,slowdown=args.collision_slowdown,stop=args.collision_stop,
                        side_warning=args.collision_side_warning,side_slowdown=args.collision_side_slowdown,side_stop=args.collision_side_stop,
                        scan_timeout=args.scan_timeout,front_degrees=args.scan_front_degrees)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.hold_seconds) or (args.hold_seconds != 0 and not 0.1 <= args.hold_seconds <= 5.0):
        parser.error("--hold-seconds 必须为0或在0.1到5.0之间")
    for name, value in (("--voice-linear", args.voice_linear), ("--voice-angular", args.voice_angular)):
        if not math.isfinite(value) or not 0 < value <= 1:
            parser.error(name + " 必须为大于0且不超过1的有限数值")
    if bool(args.cert) != bool(args.key):
        parser.error("--cert 和 --key 必须同时指定")
    if not args.no_voice and (args.model is None or not args.model.is_dir()):
        parser.error("请用 --model 指定已解压的中文模型，或用 --no-voice")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    factory = None
    if not args.no_voice:
        from vosk import Model, KaldiRecognizer
        model = Model(str(args.model))  # Load once; each phone has a separate decoder.
        grammar = None if args.free_vocabulary else command_grammar(model)
        factory = lambda: KaldiRecognizer(model, SAMPLE_RATE, grammar) if grammar else KaldiRecognizer(model, SAMPLE_RATE)
    tls = None
    if args.cert:
        tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        tls.load_cert_chain(str(args.cert), str(args.key))
    else:
        LOG.info("HTTP仅适用于localhost麦克风测试；手机采集请配置受信任的HTTPS证书")
    config = Config(cmd_topic=args.cmd_topic, imu_topic=args.imu_topic,
                    collision_enabled=not args.no_collision_guard,scan_topic=args.scan_topic,
                    collision_warning=args.collision_warning,collision_slowdown=args.collision_slowdown,
                    collision_side_warning=args.collision_side_warning,collision_side_slowdown=args.collision_side_slowdown,
                    collision_side_stop=args.collision_side_stop,
                    collision_stop=args.collision_stop,scan_timeout=args.scan_timeout,scan_front_degrees=args.scan_front_degrees,
                    voice_linear=args.voice_linear,voice_angular=args.voice_angular,
                    hold_seconds=args.hold_seconds, fast_partial=not args.confirmed_only,
                    imu_safety=args.imu_safety)
    state = ControlState(config)
    if args.start_estopped:
        state.stop(latch=True)
    publisher = PublisherThread(state, args.dry_run)
    from aiohttp import web
    publisher.start()
    try:
        web.run_app(make_app(state, factory, publisher), host=args.host, port=args.port, ssl_context=tls)
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
