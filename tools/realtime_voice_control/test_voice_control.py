"""Logic tests plus real HTTP/WebSocket tests with a deterministic ASR stub.

The stub tests routing and transport; it does not measure speech accuracy.
"""
import asyncio
import importlib.util
import json
from pathlib import Path
import struct
import sys
import time
from types import SimpleNamespace
import unittest
import uuid

TEST_DEPS = Path(__file__).parent / ".test-deps"
if TEST_DEPS.is_dir():
    sys.path.insert(0, str(TEST_DEPS))

from dual_control_voice_server import (
    AudioProtocol, AudioWorker, Config, ControlState, HEADER, IntentGate,
    PCM_BYTES, PublisherThread, StaleAudioError, make_app, parse_intent,
)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, value):
        self.now += value


class LogicTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.c = Config(voice_linear=.25, voice_angular=.2, collision_enabled=False)
        self.s = ControlState(self.c, self.clock)
        for cid in ("left", "right"):
            self.s.join(cid)
            self.s.attach_mic(cid, cid)
            self.s.audio_received(cid, cid)

    def speak(self, cid, intent):
        epoch, _ = self.s.generation(cid)
        return self.s.speech_result(cid, cid, intent, True, intent, self.clock(), epoch)

    def test_both_voice_inputs_combine_without_overwriting_peer(self):
        self.speak("left", "forward")
        self.speak("right", "forward")
        self.assertEqual(self.s.output(), (5.0, 0.0))
        self.speak("right", "backward")
        self.assertEqual(self.s.clients["left"]["value"], 0.25)
        self.assertEqual(self.s.output(), (0.0, -75.0))

    def test_turn_changes_only_assigned_slot(self):
        self.speak("left", "forward")
        self.speak("right", "forward")
        self.speak("right", "left")
        self.assertEqual(self.s.clients["left"]["value"], 0.25)
        self.assertAlmostEqual(self.s.clients["right"]["value"], 0.45)
        self.assertAlmostEqual(self.s.output()[1], 30.0)
        self.speak("left", "right")
        self.assertAlmostEqual(self.s.clients["right"]["value"], 0.45)
        self.assertLess(self.s.output()[1], 0)

    def test_ordinary_stop_only_affects_own_slot(self):
        self.speak("left", "forward")
        self.speak("right", "forward")
        self.speak("left", "stop")
        self.assertEqual(self.s.clients["right"]["value"], 0.25)
        self.assertEqual(self.s.output(), (2.5, 37.5))

    def test_estop_blocks_both_and_reset_does_not_restore_inputs(self):
        self.speak("left", "forward")
        self.speak("right", "estop")
        self.assertEqual(self.s.output(), (0.0, 0.0))
        self.assertFalse(self.speak("left", "forward"))
        with self.assertRaises(ValueError):
            self.s.command("right", 1)
        self.s.reset("left")
        self.assertEqual(self.s.output(), (0.0, 0.0))

    def test_manual_input_preserves_peer_and_invalidates_only_own_speech(self):
        self.speak("right", "forward")
        old_left, _ = self.s.generation("left")
        old_right, _ = self.s.generation("right")
        self.s.command("left", 0.5)
        self.assertEqual(self.s.generation("right")[0], old_right)
        self.assertFalse(self.s.speech_result("left", "left", "前进", True, "forward", self.clock(), old_left))
        self.assertEqual(self.s.output(), (7.5, -37.5))

    def test_old_http_command_cannot_restart_slot_after_stop(self):
        epoch, _ = self.s.generation("left")
        self.s.stop("left")
        with self.assertRaises(ValueError):
            self.s.command("left", 1.0, epoch)

    def test_voice_retains_input_during_live_silent_audio(self):
        self.speak("left", "forward")
        for _ in range(20):
            self.clock.advance(0.2)
            self.s.heartbeat("left")
            self.s.heartbeat("right")
            self.s.audio_received("left", "left")
            self.s.output()
        self.assertEqual(self.s.clients["left"]["value"], 0.25)

    def test_audio_stall_zeros_only_voice_slot(self):
        self.speak("left", "forward")
        self.s.command("right", 0.5)
        self.clock.advance(0.51)
        self.s.output()
        self.assertEqual(self.s.clients["left"]["value"], 0.0)
        self.assertEqual(self.s.clients["right"]["value"], 0.5)

    def test_optional_voice_lease_expires_independently(self):
        self.c.hold_seconds = 0.2
        self.speak("left", "forward")
        self.clock.advance(0.21)
        self.s.audio_received("left", "left")
        self.s.output()
        self.assertEqual(self.s.clients["left"]["value"], 0.0)

    def test_old_audio_cannot_move_after_stop_or_manual_takeover(self):
        stamp = self.clock()
        self.clock.advance(0.01)
        self.s.stop("left")
        epoch, _ = self.s.generation("left")
        self.assertFalse(self.s.speech_result("left", "left", "前进", True, "forward", stamp, epoch))

    def test_expired_decoding_result_rejected(self):
        stamp = self.clock()
        epoch, _ = self.s.generation("left")
        self.clock.advance(0.36)
        self.assertFalse(self.s.speech_result("left", "left", "前进", True, "forward", stamp, epoch))

    def test_disconnect_cannot_cancel_peer_speech(self):
        self.speak("left", "forward")
        self.speak("right", "backward")
        self.s.detach_mic("left", "left", "closed")
        self.assertEqual(self.s.clients["left"]["value"], 0.0)
        self.assertEqual(self.s.clients["right"]["value"], -0.25)

    def test_old_socket_cannot_clear_new_microphone(self):
        self.s.detach_mic("left", "left", "closed")
        self.s.attach_mic("left", "new-token")
        self.s.detach_mic("left", "left", "old closed late")
        self.assertEqual(self.s.clients["left"]["mic"], "new-token")

    def test_third_client_and_unjoined_commands_rejected(self):
        with self.assertRaises(ValueError):
            self.s.join("third")
        with self.assertRaises(ValueError):
            self.s.attach_mic("third", "x")
        with self.assertRaises(ValueError):
            self.s.command("third", 0.5)

    def test_heartbeat_timeout_and_single_remaining_right_slot(self):
        self.s.command("right", 0.4)
        self.clock.advance(1.01)
        self.s.heartbeat("right")
        self.assertEqual(self.s.output(), (8.0, 0.0))
        self.assertIsNone(self.s.slots["LEFT"])

    def test_single_terminal_turn_rejected(self):
        self.s.leave("right")
        self.assertFalse(self.speak("left", "left"))
        self.assertEqual(self.s.output(), (0.0, 0.0))

    def test_non_finite_inputs_cannot_reach_ros(self):
        for value in (float("nan"), float("inf"), -float("inf"), True, "0.5"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.s.command("left", value)

    def test_imu_recovery_never_restores_old_input(self):
        self.c.imu_safety = True
        self.s.update_pitch(0.0)
        self.s.command("left", 0.5)
        self.s.update_pitch(13.0)
        self.assertEqual(self.s.output(), (0.0, 0.0))
        self.s.update_pitch(0.0)
        self.assertEqual(self.s.safety, "RECOVERY")
        with self.assertRaises(ValueError):
            self.s.command("left", 0.5)
        for _ in range(6):
            self.clock.advance(0.2)
            self.s.update_pitch(0.0)
            self.s.heartbeat("left")
            self.s.heartbeat("right")
        self.assertEqual(self.s.safety, "NORMAL")
        self.assertEqual(self.s.output(), (0.0, 0.0))

    def test_imu_missing_blocks_movement(self):
        self.c.imu_safety = True
        self.assertEqual(self.s.output(), (0.0, 0.0))
        self.assertEqual(self.s.safety, "IMU_STALE")

    def test_invalid_imu_is_valid_json_null(self):
        self.s.update_pitch(float("nan"))
        json.dumps(self.s.snapshot("left"), allow_nan=False)

    def test_whole_phrase_parser_rejects_negation_conflicts_and_unlock(self):
        for text in ("不要前进", "前进然后左转", "解除急停", "恢复控制", "别停止", "今天的天气"):
            with self.subTest(text=text):
                self.assertIsNone(parse_intent(text))
        self.assertEqual(parse_intent("请 向 前 走！"), "forward")
        self.assertEqual(parse_intent("小车左转"), "left")

    def test_partial_responds_before_final_and_is_not_renewed_by_repetition(self):
        gate = IntentGate(self.c)
        self.assertIsNone(gate.feed("前进", False, 1.0))
        self.assertEqual(gate.feed("前进", False, 1.13), "forward")
        self.assertIsNone(gate.feed("前进", False, 1.4))
        self.assertIsNone(gate.feed("前进", True, 1.5))
        self.assertEqual(gate.feed("后退", True, 2.0), "backward")

    def test_confirmed_only_does_not_move_on_partial(self):
        self.c.fast_partial = False
        gate = IntentGate(self.c)
        self.assertIsNone(gate.feed("前进", False, 1.0))
        self.assertIsNone(gate.feed("前进", False, 2.0))
        self.assertEqual(gate.feed("前进", True, 2.1), "forward")
        self.assertEqual(gate.feed("急停", False, 2.2), "estop")

    def test_continuous_short_commands_turn_and_stop_without_waiting_for_final(self):
        gate = IntentGate(self.c)
        self.assertIsNone(gate.feed("前进", False, 1.0))
        self.assertEqual(gate.feed("前进", False, 1.09), "forward")
        self.assertIsNone(gate.feed("前进 左转", False, 1.1))
        self.assertEqual(gate.feed("前进 左转", False, 1.19), "left")
        self.assertIsNone(gate.feed("前进 左转", False, 1.3))
        self.assertEqual(gate.feed("前进 左转 停止", False, 1.31), "stop")
        self.assertIsNone(gate.feed("前进 左转 停止", True, 1.4))

    def test_revision_of_already_executed_word_cannot_reverse_direction(self):
        gate = IntentGate(self.c)
        gate.feed("前进", False, 1.0)
        self.assertEqual(gate.feed("前进", False, 1.09), "forward")
        self.assertIsNone(gate.feed("后退", False, 1.1))
        self.assertIsNone(gate.feed("后退", False, 1.3))
        self.assertIsNone(gate.feed("后退", True, 1.4))
        self.assertEqual(gate.feed("后退", True, 1.6), "backward")

    def test_speed_command_negation_is_rejected(self):
        for text in ("不要加速", "别快点", "不许减速", "取消前进", "前进然后加速"):
            gate = IntentGate(self.c)
            self.assertIsNone(gate.feed(text, True, 1))
        self.assertEqual(parse_intent("快一点"), "faster")
        self.assertEqual(parse_intent("慢点"), "slower")

    def test_stop_anywhere_in_command_sequence_takes_priority_over_movement(self):
        for text, expected in (("停止前进", "stop"), ("前进停止后退", "stop"), ("急停前进", "estop"), ("前进急停左转", "estop")):
            with self.subTest(text=text):
                self.assertEqual(IntentGate(self.c).feed(text,False,1), expected)

    def test_unknown_acoustic_token_can_only_be_skipped_for_stop(self):
        self.assertEqual(IntentGate(self.c).feed('[unk] 停止',False,1),'stop')
        self.assertEqual(IntentGate(self.c).feed('前进 [unk] 急停',False,1),'estop')
        self.assertIsNone(IntentGate(self.c).feed('[unk] 前进',True,1))
        self.assertIsNone(IntentGate(self.c).feed('不要 [unk] 停止',True,1))

    def test_passive_motion_and_scan_feedback_does_not_change_command(self):
        self.s.command("left", .5)
        before = self.s.output()
        self.s.update_odom(0.0, 0.0)
        scan = SimpleNamespace(angle_min=-.1,angle_increment=.1,range_min=.05,range_max=10,
                               ranges=[float('nan'),.12,float('inf')])
        self.s.update_scan(scan)
        self.assertEqual(self.s.output(), before)
        self.assertEqual(self.s.snapshot("left")["feedback"]["front_distance_m"], .12)
        self.assertEqual(self.s.snapshot("left")["feedback"]["odom"], (0,0))
        self.clock.advance(1.01)
        self.assertIsNone(self.s.snapshot("left")["feedback"]["odom"])
        self.assertIsNone(self.s.snapshot("left")["feedback"]["front_distance_m"])

    def test_protocol_rejects_delayed_wrong_size_and_missing_sequence(self):
        protocol = AudioProtocol(self.c, self.clock)
        tick = protocol.tick()
        pcm = bytes(PCM_BYTES)
        self.assertEqual(protocol.decode(HEADER.pack(tick, 0) + pcm), (pcm, tick))
        with self.assertRaises(ValueError):
            protocol.decode(HEADER.pack(tick, 2) + pcm)
        with self.assertRaises(ValueError):
            protocol.decode(b"too short")
        self.clock.advance(0.36)
        with self.assertRaises(ValueError):
            protocol.decode(HEADER.pack(tick, 1) + pcm)

    def test_queue_overflow_discards_old_audio_without_closing_or_changing_peer(self):
        self.speak("left", "forward")
        self.speak("right", "backward")
        worker = AudioWorker(self.s, "left", "left", lambda: None)
        for _ in range(12):
            worker.put(bytes(PCM_BYTES), self.clock())
        old_epoch, _ = self.s.generation("left")
        worker.put(bytes(PCM_BYTES), self.clock())
        self.assertFalse(worker.closed.is_set())
        self.assertTrue(self.s.mic_active("left", "left"))
        self.assertEqual(worker.queue.qsize(), 1)
        self.assertEqual(self.s.clients["left"]["value"], 0.0)
        self.assertEqual(self.s.clients["right"]["value"], -.25)
        self.assertFalse(self.s.speech_result("left", "left", "前进", True, "forward", self.clock(), old_epoch))

    def test_dropped_network_audio_advances_sequence_and_allows_fresh_audio(self):
        protocol = AudioProtocol(self.c, self.clock)
        tick = protocol.tick()
        self.clock.advance(.36)
        with self.assertRaises(StaleAudioError):
            protocol.decode(HEADER.pack(tick, 0) + bytes(PCM_BYTES))
        fresh = protocol.tick()
        self.assertEqual(protocol.decode(HEADER.pack(fresh, 1) + bytes(PCM_BYTES))[1], fresh)


class RoleTests(unittest.TestCase):
    def setUp(self):
        self.s = ControlState(Config(voice_linear=.25, voice_angular=.2, collision_enabled=False))
        self.s.connect("a")
        self.s.connect("b")

    def speak(self, cid, intent):
        if not self.s.clients[cid]["mic"]:
            self.s.attach_mic(cid, cid)
        self.s.audio_received(cid, cid)
        epoch, _ = self.s.generation(cid)
        return self.s.speech_result(cid, cid, intent, True, intent, self.s.clock(), epoch)

    def test_connection_does_not_assign_role_or_allow_motion(self):
        self.assertIsNone(self.s.snapshot("a")["my_slot"])
        with self.assertRaises(ValueError):
            self.s.axis_command("a", .25, .2)
        with self.assertRaises(ValueError):
            self.s.attach_mic("a", "a")

    def test_full_voice_controls_both_axes_on_one_phone(self):
        self.s.claim("a", "FULL")
        self.assertTrue(self.speak("a", "forward"))
        self.assertEqual(self.s.output(), (5.0, 0.0))
        self.assertTrue(self.speak("a", "left"))
        self.assertEqual(self.s.output(), (0.0, 60.0))
        self.s.axis_command("a", .25, -.2)
        self.assertEqual(self.s.output(), (5.0, -60.0))

    def test_role_exclusivity_is_enforced_on_server(self):
        self.s.claim("a", "FULL")
        for role in ("FULL", "LINEAR", "ANGULAR", "LEFT"):
            with self.subTest(role=role), self.assertRaises(ValueError):
                self.s.claim("b", role)
        self.s.release_role("a")
        self.s.claim("b", "ANGULAR")
        self.s.claim("a", "LINEAR")
        with self.assertRaises(ValueError):
            self.s.claim("a", "FULL")
        with self.assertRaises(ValueError):
            self.s.claim("a", "RIGHT")

    def test_dual_voice_scopes_axes_and_stop_preserves_peer(self):
        self.s.claim("a", "LINEAR")
        self.s.claim("b", "ANGULAR")
        self.speak("a", "forward")
        self.speak("b", "left")
        self.assertEqual(self.s.output(), (5.0, 60.0))
        self.assertFalse(self.speak("a", "right"))
        self.assertFalse(self.speak("b", "backward"))
        self.assertEqual(self.s.output(), (5.0, 60.0))
        self.speak("b", "stop")
        self.assertEqual(self.s.output(), (5.0, 0.0))
        self.speak("a", "estop")
        self.assertEqual(self.s.output(), (0.0, 0.0))

    def test_wrong_manual_axis_is_rejected_and_values_are_clamped(self):
        self.s.claim("a", "LINEAR")
        self.s.claim("b", "ANGULAR")
        with self.assertRaises(ValueError):
            self.s.axis_command("a", .2, .1)
        with self.assertRaises(ValueError):
            self.s.axis_command("b", .1, .2)
        self.s.axis_command("a", 5, 0)
        self.s.axis_command("b", 0, -5)
        self.assertEqual(self.s.output(), (20.0, -300.0))

    def test_role_release_and_reassignment_clear_old_mic_and_input(self):
        self.s.claim("a", "FULL")
        self.speak("a", "forward")
        old_epoch, _ = self.s.generation("a")
        self.s.release_role("a")
        self.assertTrue(self.s.snapshot("a")["connected"])
        self.assertFalse(self.s.mic_active("a", "a"))
        self.s.claim("a", "ANGULAR")
        self.assertEqual(self.s.output(), (0.0, 0.0))
        self.assertFalse(self.s.speech_result("a", "a", "前进", True, "forward", self.s.clock(), old_epoch))

    def test_speed_steps_are_bounded_and_only_change_own_assigned_axis(self):
        self.s.claim("a", "LINEAR")
        self.s.claim("b", "ANGULAR")
        self.assertFalse(self.speak("a", "faster"))
        self.assertEqual(self.s.output(), (0,0))
        self.speak("a", "forward")
        self.speak("b", "left")
        self.speak("a", "faster")
        self.assertEqual(self.s.output(), (7,60))
        for _ in range(12):
            self.speak("a", "faster")
        self.assertEqual(self.s.output(), (13,60))
        self.assertIn("上限", self.s.snapshot("a")["voice_feedback"])
        for _ in range(12):
            self.speak("a", "slower")
        self.assertEqual(self.s.output(), (3,60))
        self.assertIn("下限", self.s.snapshot("a")["voice_feedback"])
        self.speak("a", "stop")
        self.assertFalse(self.speak("a", "faster"))
        self.assertEqual(self.s.output(), (0,60))

    def test_full_speed_keeps_direction_and_reset_restores_initial_gear(self):
        self.s.claim("a", "FULL")
        self.speak("a", "backward")
        self.speak("a", "faster")
        self.assertEqual(self.s.output(), (-7,0))
        self.speak("a", "left")
        self.speak("a", "faster")
        self.assertAlmostEqual(self.s.output()[1], 90)
        self.speak("a", "estop")
        self.s.reset("a")
        self.speak("a", "forward")
        self.assertEqual(self.s.output(), (5,0))

    def test_deceleration_never_increases_a_tiny_manual_input(self):
        self.s.claim("a", "FULL")
        self.s.axis_command("a", .05, 0)
        self.assertFalse(self.speak("a", "slower"))
        self.assertEqual(self.s.output(), (1,0))


class StubRecognizer:
    """First two samples encode a transcript and final/partial flag."""
    def AcceptWaveform(self, pcm):
        code, final = struct.unpack_from("<hh", pcm)
        self.text = {1:"前进",2:"后退",3:"左转",4:"右转",5:"停止",6:"急停",7:"不要前进",
                     8:"加速",9:"减速",10:"前进 左转",11:"前进 左转 停止",12:"前进 加速"}.get(code, "")
        return bool(final)

    def Result(self):
        return json.dumps(dict(text=self.text))

    def PartialResult(self):
        return json.dumps(dict(partial=self.text))

    def Reset(self):
        self.text = ""


@unittest.skipUnless(importlib.util.find_spec("aiohttp"), "install aiohttp to run HTTP/WebSocket integration tests")
class NetworkTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        from aiohttp.test_utils import TestClient, TestServer
        self.state = ControlState(Config(voice_linear=.25, voice_angular=.2, collision_enabled=False))
        self.publisher = PublisherThread(self.state, dry_run=True)
        self.publisher.start()
        self.client = TestClient(TestServer(make_app(self.state, StubRecognizer, self.publisher)))
        await self.client.start_server()
        self.sockets = []
        for cid in ("left", "right"):
            self.assertEqual((await self.post("join", cid)).status, 200)

    async def asyncTearDown(self):
        for ws in self.sockets:
            await ws.close()
        await self.client.close()
        self.publisher.close()

    async def post(self, action, cid, **extra):
        return await self.client.post("/" + action, json=dict(client_id=cid, **extra))

    async def microphone(self, cid):
        ws = await self.client.ws_connect("/audio?client_id=" + cid)
        self.sockets.append(ws)
        tick = await ws.receive_json()
        return dict(ws=ws, tick=tick["stamp"], seq=0, cid=cid)

    async def frame(self, stream, code, final=True):
        after = self.state.generation(stream["cid"])[1]
        while time.monotonic() - stream["tick"] > 0.15 or stream["tick"] < after:
            stream["tick"] = (await stream["ws"].receive_json())["stamp"]
        pcm = struct.pack("<hh", code, int(final)) + bytes(PCM_BYTES - 4)
        await stream["ws"].send_bytes(HEADER.pack(stream["tick"], stream["seq"]) + pcm)
        stream["seq"] += 1

    async def until(self, predicate):
        for _ in range(80):
            if predicate():
                return
            await asyncio.sleep(0.01)
        self.fail("asynchronous state did not reach expected value")

    async def test_two_real_websockets_feed_separate_slot_recognizers(self):
        left, right = await self.microphone("left"), await self.microphone("right")
        await self.frame(left, 1)
        await self.frame(right, 2)
        await self.until(lambda: self.state.clients["left"]["value"] == .25 and self.state.clients["right"]["value"] == -.25)
        self.assertEqual(self.state.output(), (0.0, -75.0))

    async def test_partial_command_reaches_output_without_final_result(self):
        stream = await self.microphone("left")
        for _ in range(8):
            await self.frame(stream, 1, final=False)
            await asyncio.sleep(0.025)
        await self.until(lambda: self.state.clients["left"]["value"] == .25)
        self.assertEqual(self.state.clients["left"]["speech_kind"], "partial")

    async def test_escape_http_permissions_lease_release_and_peer_stop(self):
        for cid,role in [('left','LINEAR'),('right','ANGULAR')]:
            await self.post('release-role',cid)
        await self.post('claim','left',role='LINEAR')
        await self.post('claim','right',role='ANGULAR')
        token='http-escape-token'
        epoch=self.state.generation('left')[0]
        self.assertEqual((await self.post('escape','left',phase='start',token=token,direction='left',epoch=epoch)).status,400)
        result=await self.post('escape','left',phase='start',token=token,direction='backward',epoch=epoch)
        self.assertEqual(result.status,200)
        self.assertEqual((await result.json())['last_cmd']['linear_x'],-4)
        self.assertEqual((await self.post('escape','right',phase='keep',token=token)).status,400)
        await self.post('stop','right');self.assertEqual(self.state.output(),(0,0))
        await self.post('escape','left',phase='release',token=token)
        epoch=self.state.generation('left')[0]
        self.assertEqual((await self.post('escape','left',phase='start',token=token,direction='forward',epoch=epoch)).status,400)

    async def test_manual_mode_http_keeps_voice_and_touch_usable_without_scan(self):
        from collision_guard import CollisionGuard
        self.state.collision=CollisionGuard(clock=self.state.clock)
        await self.post('release-role','left');await self.post('release-role','right')
        await self.post('claim','left',role='LINEAR');await self.post('claim','right',role='ANGULAR')
        result=await self.post('escape-mode','left',enabled=True,epoch=self.state.generation('left')[0])
        self.assertEqual(result.status,200)
        self.assertTrue((await result.json())['driving_mode']['escape'])
        stream=await self.microphone('left')
        await self.frame(stream,1)
        await self.until(lambda:self.state.output()[0]>0)
        result=await self.post('cmd','right',angular=.5,epoch=self.state.generation('right')[0])
        self.assertEqual(result.status,200);self.assertEqual(self.state.output(),(5,150))
        self.assertFalse(self.state.collision.fresh())
        result=await self.post('escape-mode','right',enabled=False,epoch=self.state.generation('right')[0])
        self.assertEqual(result.status,200);self.assertEqual(self.state.output(),(0,0))

    async def test_same_utterance_speed_step_is_not_repeated_for_every_partial(self):
        stream = await self.microphone("left")
        for code in (1,12):
            for _ in range(6):
                await self.frame(stream, code, final=False)
                await asyncio.sleep(.025)
        await self.until(lambda:self.state.clients['left']['value']==.35)
        self.assertEqual(self.state.clients['left']['value'], .35)

    async def test_same_socket_continuous_words_can_stop_without_a_sentence_final(self):
        stream = await self.microphone("left")
        for code in (1,10):
            for _ in range(6):
                await self.frame(stream, code, final=False)
                await asyncio.sleep(.025)
        await self.until(lambda:self.state.clients['left']['last_intent']=='left')
        await self.frame(stream,11,final=False)
        await self.until(lambda:self.state.clients['left']['value']==0)
        self.assertEqual(self.state.clients['left']['last_intent'], 'stop')

    async def test_websocket_disconnect_zeros_only_its_slot(self):
        left, right = await self.microphone("left"), await self.microphone("right")
        await self.frame(left, 1)
        await self.frame(right, 2)
        await self.until(lambda: self.state.clients["left"]["value"] == .25 and self.state.clients["right"]["value"] == -.25)
        async def keep_peer_live():
            while True:
                await self.frame(right, 0, final=False)
                self.state.heartbeat("left")
                self.state.heartbeat("right")
                await asyncio.sleep(0.02)
        live = asyncio.create_task(keep_peer_live())
        try:
            # Abrupt network loss rather than waiting for a close handshake.
            left["ws"]._response.connection.transport.abort()
            await self.until(lambda: not self.state.snapshot("left")["mic_connected"])
            self.assertEqual(self.state.clients["left"]["value"], 0)
            self.assertEqual(self.state.clients["right"]["value"], -.25)
        finally:
            live.cancel()
            await asyncio.gather(live, return_exceptions=True)

    async def test_voice_stop_endpoint_clears_only_own_voice_input(self):
        left, right = await self.microphone("left"), await self.microphone("right")
        await self.frame(left, 1)
        await self.frame(right, 1)
        await self.until(lambda: self.state.output() == (5.0, 0.0))
        response = await self.post("voice-stop", "left")
        self.assertEqual(response.status, 200)
        self.assertEqual(self.state.clients["left"]["value"], 0)
        self.assertEqual(self.state.clients["right"]["value"], .25)

    async def test_stale_audio_is_dropped_and_same_socket_accepts_fresh_command(self):
        stream = await self.microphone("left")
        await asyncio.sleep(0.37)
        await stream["ws"].send_bytes(HEADER.pack(stream["tick"], 0) + bytes(PCM_BYTES))
        stream["seq"] = 1
        await self.until(lambda: self.state.clients["left"]["dropped_frames"] > 0)
        self.assertTrue(self.state.snapshot("left")["mic_connected"])
        self.assertEqual(self.state.clients["left"]["value"], 0)
        await self.frame(stream, 1)
        await self.until(lambda: self.state.clients["left"]["value"] == .25)

    async def test_connect_choose_role_and_two_axis_commands(self):
        for cid in ("left", "right"):
            await self.post("leave", cid)
            r = await self.post("connect", cid)
            self.assertIsNone((await r.json())["my_slot"])
        self.assertEqual((await self.post("claim", "left", role="LINEAR")).status, 200)
        self.assertEqual((await self.post("claim", "right", role="ANGULAR")).status, 200)
        for cid, v, w in (("left", .25, 0), ("right", 0, -.2)):
            epoch, _ = self.state.generation(cid)
            self.assertEqual((await self.post("cmd", cid, linear=v, angular=w, epoch=epoch)).status, 200)
        self.assertEqual(self.state.output(), (5.0, -60.0))
        epoch, _ = self.state.generation("left")
        self.assertEqual((await self.post("cmd", "left", linear=0, angular=.3, epoch=epoch)).status, 400)
        status = await self.client.get("/state?client_id=left")
        self.assertEqual((await status.json())["my_slot"], "LINEAR")

    async def test_api_rejects_non_object_and_non_finite_values(self):
        response = await self.client.post("/cmd", json=[])
        self.assertEqual(response.status, 400)
        response = await self.post("cmd", "left", value=float("nan"), epoch=0)
        self.assertEqual(response.status, 400)

    async def test_third_phone_has_no_slot_or_audio_permission(self):
        response = await self.post("join", "third")
        self.assertEqual(response.status, 400)
        response = await self.client.get("/audio?client_id=third")
        self.assertEqual(response.status, 403)

    async def test_voice_stop_is_local_estop_is_global(self):
        left, right = await self.microphone("left"), await self.microphone("right")
        await self.frame(left, 1)
        await self.frame(right, 1)
        await self.until(lambda: self.state.output() == (5.0, 0.0))
        await self.frame(left, 5)
        await self.until(lambda: self.state.clients["left"]["value"] == 0)
        self.assertEqual(self.state.clients["right"]["value"], .25)
        await self.frame(right, 6)
        await self.until(lambda: self.state.estop)
        self.assertEqual(self.state.output(), (0.0, 0.0))

    async def test_static_assets_and_health(self):
        for path in ("/", "/pcm-worklet.js", "/collision-alerts.js", "/control-session.js", "/health"):
            response = await self.client.get(path)
            self.assertEqual(response.status, 200)
        response = await self.client.get("/health")
        self.assertTrue((await response.json())["voice"])

    async def test_collision_alarm_is_shared_and_both_voice_slots_are_stopped(self):
        from collision_guard import CollisionConfig, CollisionGuard
        self.state.collision=CollisionGuard(CollisionConfig(),self.state.clock)
        self.state.update_scan(SimpleNamespace(range_min=.05,range_max=12.,angle_min=0.,
            angle_increment=1.5707963267948966,ranges=[1.,1.,1.,1.]))
        left,right=await self.microphone('left'),await self.microphone('right')
        await self.frame(left,1)
        await self.frame(right,1)
        await self.until(lambda:self.state.output()==(5.,0.))
        self.state.update_scan(SimpleNamespace(range_min=.05,range_max=12.,angle_min=0.,
            angle_increment=1.5707963267948966,ranges=[.2,1.,1.,1.]))
        a=await (await self.post('heartbeat','left')).json()
        b=await (await self.post('heartbeat','right')).json()
        self.assertEqual(a['collision']['last_event']['id'],b['collision']['last_event']['id'])
        self.assertEqual(a['last_cmd'],dict(linear_x=0.,angular_z=0.))
        self.assertEqual(a['my_input']['value'],0)
        self.assertEqual(b['my_input']['value'],0)

    async def test_collision_blocks_touch_and_rejects_its_old_request_epoch(self):
        from collision_guard import CollisionConfig, CollisionGuard
        self.state.collision=CollisionGuard(CollisionConfig(),self.state.clock)
        self.state.update_scan(SimpleNamespace(range_min=.05,range_max=12.,angle_min=0.,
            angle_increment=1.5707963267948966,ranges=[.2,1.,1.,1.]))
        epoch,_=self.state.generation('left')
        result=await (await self.post('cmd','left',value=.5,epoch=epoch)).json()
        self.assertEqual(result['last_cmd'],dict(linear_x=0.,angular_z=0.))
        self.assertTrue(result['collision']['awaiting_new_command'])
        self.assertEqual((await self.post('cmd','left',value=.5,epoch=epoch)).status,400)


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "source ROS 2 to run the real publisher regression test")
class RosPublisherTests(unittest.TestCase):
    def test_custom_context_thread_publishes_dual_axes_and_estop_on_isolated_topic(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        topic = "/voice_deployment_test_" + uuid.uuid4().hex
        context = Context()
        rclpy.init(context=context)
        observer = Node("voice_deployment_observer", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(observer)
        received = []
        observer.create_subscription(Twist, topic, lambda msg: received.append((msg.linear.x, msg.angular.z)), 10)
        state = ControlState(Config(cmd_topic=topic, imu_topic=topic + "_imu", collision_enabled=False))
        publisher = PublisherThread(state)
        try:
            publisher.start()
            deadline = time.monotonic() + 4.0
            while len(received) < 5 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if publisher.error:
                    raise publisher.error
            self.assertGreaterEqual(len(received), 5)
            self.assertTrue(all(command == (0.0, 0.0) for command in received))
            for cid, role in (("a", "LINEAR"), ("b", "ANGULAR")):
                state.connect(cid)
                state.claim(cid, role)
            for cid, intent in (("a", "forward"), ("b", "right")):
                state.attach_mic(cid, cid)
                state.audio_received(cid, cid)
                epoch, _ = state.generation(cid)
                self.assertTrue(state.speech_result(cid, cid, intent, True, intent, state.clock(), epoch))
            def speak(cid,intent):
                epoch,_ = state.generation(cid)
                self.assertTrue(state.speech_result(cid,cid,intent,True,intent,state.clock(),epoch))
            for expected, action in (((7.0, -150.0), lambda: None),
                                     ((9.0, -150.0), lambda:speak('a','faster')),
                                     ((7.0, -150.0), lambda:speak('a','slower')),
                                     ((7.0, 0.0), lambda: state.stop("b")),
                                     ((0.0, 0.0), lambda: state.stop("a", latch=True))):
                received.clear()
                action()
                deadline = time.monotonic() + .8
                while expected not in received and time.monotonic() < deadline:
                    state.heartbeat("a")
                    state.heartbeat("b")
                    state.audio_received("a", "a")
                    state.audio_received("b", "b")
                    executor.spin_once(timeout_sec=.02)
                self.assertIn(expected, received)
        finally:
            publisher.close()
            executor.shutdown(timeout_sec=1.0)
            observer.destroy_node()
            context.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
