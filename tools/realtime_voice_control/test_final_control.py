"""Regression tests for held escape, priority, raw scans and communication loss."""
import importlib.util
import math
import struct
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch,Mock
from dual_control_voice_server import Config, ControlState,CommandCadence
from scan_decoder import decode_scan
from test_collision_guard import Clock,scan


def serialized_scan(endian='<',intensities=()):
    frame=b'laser_frame\0'
    data=bytearray(b'\0\1\0\0' if endian=='<' else b'\0\0\0\0')
    data+=struct.pack(endian+'iII',5,7,len(frame))+frame
    data+=bytes((-(len(data)-4))%4)
    data+=struct.pack(endian+'7fI',0,3*math.pi/2,math.pi/2,0,.1,.05,12,4)
    data+=struct.pack(endian+'4f',1,2,3,4)
    data+=struct.pack(endian+'I',len(intensities))
    data+=struct.pack(endian+str(len(intensities))+'f',*intensities)
    return bytes(data)


class FinalControlTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.s=ControlState(Config(),self.clock)
        for cid,role in [('a','LINEAR'),('b','ANGULAR')]:
            self.s.connect(cid);self.s.claim(cid,role)
        self.token='test-escape-token'

    def start(self,cid='a',direction='forward',token=None):
        self.s.escape(cid,'start',token or self.token,direction,self.s.generation(cid)[0])

    def test_normal_radar_fail_closed_explicit_escape_is_limited_and_release_clears(self):
        self.s.axis_command('a',.5,0)
        self.assertEqual(self.s.output(),(0,0))
        self.start();self.assertEqual(self.s.output(),(4,0))
        self.s.escape('a','release',self.token)
        self.assertEqual(self.s.output(),(0,0))
        self.s.update_scan(scan());self.assertEqual(self.s.output(),(0,0))

    def test_missing_keepalive_stops_within_lease_and_late_renew_cannot_restart(self):
        self.start();self.clock.advance(.251)
        self.assertEqual(self.s.output(),(0,0))
        with self.assertRaises(ValueError):self.s.escape('a','keep',self.token)
        with self.assertRaises(ValueError):self.start()

    def test_continuous_hold_has_no_fixed_deadline_but_release_still_stops(self):
        self.start()
        for _ in range(300):
            self.clock.advance(.1);self.s.escape('a','keep',self.token)
        self.assertEqual(self.s.output(),(4,0))
        self.assertIsNone(self.s.takeover.snapshot()['maximum_seconds'])
        self.s.escape('a','release',self.token)
        self.assertEqual(self.s.output(),(0,0))

    def test_release_reordered_before_start_cannot_initiate_motion(self):
        self.s.escape('a','release',self.token)
        with self.assertRaises(ValueError):self.start()
        self.assertEqual(self.s.output(),(0,0))

    def test_invalid_tokens_and_old_epochs_rejected(self):
        for token in (None,True,{},'x'):
            with self.subTest(token=token),self.assertRaises(ValueError):self.s.escape('a','release',token)
        epoch=self.s.generation('a')[0];self.s.stop('a')
        with self.assertRaises(ValueError):self.s.escape('a','start',self.token,'forward',epoch)

    def test_role_limits_and_other_phone_cannot_renew_or_cancel_owner(self):
        with self.assertRaises(ValueError):self.start(direction='left')
        self.start()
        with self.assertRaises(ValueError):self.s.escape('b','keep',self.token)
        self.s.escape('b','release',self.token)
        self.assertEqual(self.s.output(),(4,0))
        self.s.stop('b');self.assertEqual(self.s.output(),(0,0))

    def test_peer_motion_cannot_mix_with_escape_but_peer_estop_always_wins(self):
        self.start()
        with self.assertRaises(ValueError):self.s.axis_command('b',0,.8)
        self.s.stop('b',latch=True)
        self.assertEqual(self.s.output(),(0,0))
        with self.assertRaises(ValueError):self.start(token='another-token')
        self.s.reset('b');self.assertEqual(self.s.output(),(0,0))

    def test_release_role_disconnect_and_role_switch_cancel_immediately(self):
        for action in ('release','leave','claim'):
            self.setUp();self.start()
            if action=='release':self.s.release_role('a')
            elif action=='leave':self.s.leave('a')
            else:
                self.s.release_role('b');self.s.claim('a','FULL')
            self.assertIsNone(self.s.takeover.current)
            self.assertEqual(self.s.output(),(0,0))

    def test_imu_protection_is_never_bypassed(self):
        self.s.c.imu_safety=True;self.s.update_pitch(0)
        self.start();self.s.update_pitch(13)
        self.assertEqual(self.s.output(),(0,0))
        with self.assertRaises(ValueError):self.start(token='another-token')

    def test_imu_timeout_stops_escape(self):
        self.s.c.imu_safety=True;self.s.update_pitch(0);self.start()
        for _ in range(5):
            self.clock.advance(.1);self.s.escape('a','keep',self.token)
        self.clock.advance(.01);self.assertEqual(self.s.output(),(0,0))

    def test_agent_crash_latches_stop_and_recovery_never_resumes(self):
        self.s.output_mode='ROS2';self.s.update_agent_presence(True);self.start()
        self.s.update_agent_presence(False)
        self.assertTrue(self.s.estop);self.assertEqual(self.s.agent_disconnects,1)
        self.s.reset('a')
        with self.assertRaises(ValueError):self.start(token='another-token')
        self.s.update_agent_presence(True);self.assertEqual(self.s.output(),(0,0))

    def test_touch_priority_is_local_and_new_speech_after_grace_can_control(self):
        self.s.update_scan(scan())
        for cid in ('a','b'):
            self.s.attach_mic(cid,cid);self.s.audio_received(cid,cid)
        self.s.axis_command('a',.2,0)
        def say(cid,intent):
            self.s.audio_received(cid,cid)
            return self.s.speech_result(cid,cid,intent,True,intent,self.clock(),self.s.generation(cid)[0])
        self.assertFalse(say('a','backward'))
        self.assertTrue(say('b','left'))
        self.assertEqual(self.s.output()[0],4)
        self.clock.advance(.61);self.s.update_scan(scan());self.s.heartbeat('b')
        self.assertTrue(say('a','backward'));self.assertLess(self.s.output()[0],0)

    def test_voice_stop_interrupts_escape_even_during_touch_priority(self):
        self.s.attach_mic('b','b');self.s.audio_received('b','b');self.start()
        epoch=self.s.generation('b')[0]
        self.assertTrue(self.s.speech_result('b','b','停止',True,'stop',self.clock(),epoch))
        self.assertEqual(self.s.output(),(0,0))


class EscapeModeTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.s=ControlState(Config(),self.clock)
        for cid,role in [('a','LINEAR'),('b','ANGULAR')]:
            self.s.connect(cid);self.s.claim(cid,role)

    def mode(self,enabled,cid='a'):
        self.s.set_escape_mode(cid,enabled,self.s.generation(cid)[0])

    def test_no_scan_touch_both_axes_are_not_gated_by_radar(self):
        self.s.axis_command('a',.35,0);self.assertEqual(self.s.output(),(0,0))
        self.mode(True)
        self.s.axis_command('a',.35,0);self.s.axis_command('b',0,.5)
        self.assertEqual(self.s.output(),(7,150))
        self.assertFalse(self.s.collision.fresh())
        self.assertTrue(self.s.snapshot('b')['driving_mode']['escape'])

    def test_no_scan_speech_and_role_isolation_remain_usable(self):
        self.mode(True)
        def speak(cid,intent):
            self.s.attach_mic(cid,cid);self.s.audio_received(cid,cid)
            return self.s.speech_result(cid,cid,intent,True,intent,self.clock(),self.s.generation(cid)[0])
        self.assertTrue(speak('a','forward'));self.assertTrue(speak('b','left'))
        self.assertEqual(self.s.output(),(7,150))
        with self.assertRaises(ValueError):self.s.axis_command('a',0,.5)

    def test_close_bad_and_expired_scan_are_displayed_but_do_not_limit_escape(self):
        self.mode(True);self.s.axis_command('a',.5,0);self.s.axis_command('b',0,.5)
        self.s.update_scan(scan(.08,.08,.08,.08));self.assertEqual(self.s.output(),(10,150))
        self.assertAlmostEqual(self.s.snapshot('a')['collision']['distances']['front'],.08)
        self.s.update_scan_bytes(b'invalid');self.assertEqual(self.s.output(),(10,150))
        self.clock.advance(.6);self.assertEqual(self.s.output(),(10,150))

    def test_leaving_mode_zeros_old_commands_and_restores_scan_gate(self):
        self.mode(True);self.s.axis_command('a',.35,0)
        old=self.s.generation('a')[0];self.mode(False,'b')
        self.assertEqual(self.s.output(),(0,0))
        with self.assertRaises(ValueError):self.s.axis_command('a',.35,0,old)
        self.s.axis_command('a',.35,0);self.assertEqual(self.s.output(),(0,0))
        self.s.update_scan(scan());self.assertEqual(self.s.output(),(0,0))
        self.s.axis_command('a',.35,0);self.assertEqual(self.s.output(),(7,0))

    def test_estop_agent_and_imu_remain_enforced(self):
        self.mode(True);self.s.stop('b',latch=True)
        with self.assertRaises(ValueError):self.s.axis_command('a',.35,0)
        self.s.reset('a');self.s.output_mode='ROS2';self.s.update_agent_presence(False)
        with self.assertRaises(ValueError):self.s.axis_command('a',.35,0)
        self.s.update_agent_presence(True);self.s.c.imu_safety=True;self.s.update_pitch(13)
        with self.assertRaises(ValueError):self.s.axis_command('a',.35,0)

    def test_mode_owner_disconnect_or_release_stops_peer_and_restores_protection(self):
        for action in ('leave','release_role'):
            self.setUp();self.mode(True);self.s.axis_command('b',0,.5)
            getattr(self.s,action)('a')
            self.assertFalse(self.s.snapshot('b')['driving_mode']['escape'])
            self.assertEqual(self.s.output(),(0,0))

    def test_mode_owner_heartbeat_timeout_stops_both(self):
        self.mode(True);self.s.axis_command('b',0,.5)
        self.clock.advance(1.01);self.s.heartbeat('b')
        self.assertEqual(self.s.output(),(0,0));self.assertIsNone(self.s.escape_mode_owner)

    def test_invalid_mode_or_old_epoch_cannot_enable_bypass(self):
        epoch=self.s.generation('a')[0];self.s.stop('a')
        with self.assertRaises(ValueError):self.s.set_escape_mode('a',True,epoch)
        for value in ('true',1,None):
            with self.assertRaises(ValueError):self.s.set_escape_mode('a',value,self.s.generation('a')[0])
        with self.assertRaises(ValueError):self.s.set_escape_mode('unknown',True,1)


class ScanDecoderTests(unittest.TestCase):
    def test_both_endian_and_unused_intensity_array(self):
        for endian in ('<','>'):
            for intensities in ((),(1.,2.,3.,4.)):
                result=decode_scan(serialized_scan(endian,intensities))
                self.assertEqual(result.ranges,(1.,2.,3.,4.))
                self.assertEqual(result.header.frame_id,'laser_frame')
                self.assertFalse(hasattr(result,'intensities'))

    def test_truncated_malicious_lengths_and_encoding_are_rejected(self):
        original=serialized_scan()
        invalid=[original[:n] for n in range(len(original))]
        invalid += [original+b'\0'*4,b'\0\3\0\0'+original[4:],
                    original[:12]+struct.pack('<I',2**32-1)+original[16:],
                    original[:-4]+struct.pack('<I',2**32-1)]
        for data in invalid:
            with self.subTest(size=len(data)),self.assertRaises((ValueError,UnicodeError)):decode_scan(data)

    def test_rejected_frame_clears_motion_and_is_reported(self):
        s=ControlState(Config());s.connect('a');s.claim('a','FULL')
        s.update_scan_bytes(serialized_scan());s.axis_command('a',.3,0)
        self.assertEqual(s.output(),(6,0))
        s.update_scan_bytes(b'bad frame');self.assertEqual(s.output(),(0,0))
        self.assertEqual(s.snapshot('a')['scan_diagnostics']['rejected_frames'],1)
        s.update_scan_bytes(serialized_scan());self.assertEqual(s.output(),(0,0))

    @unittest.skipUnless(importlib.util.find_spec('rclpy'),'ROS2 serialization required')
    def test_actual_humble_serialization(self):
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import LaserScan
        message=LaserScan();message.header.frame_id='laser'
        message.range_min=.05;message.range_max=12.;message.angle_increment=.1
        message.ranges=[.25,1.,float('inf')];message.intensities=[2.,3.,4.]
        result=decode_scan(serialize_message(message))
        self.assertEqual(result.ranges[0],.25)
        self.assertTrue(math.isinf(result.ranges[2]))
        self.assertEqual(result.header.frame_id,'laser')


class AgentTests(unittest.TestCase):
    def test_changed_commands_and_stop_are_immediate_steady_stream_is_reduced(self):
        clock=Clock();cadence=CommandCadence(clock)
        self.assertTrue(cadence.due((7,0)))
        clock.advance(.02);self.assertFalse(cadence.due((7,0)))
        self.assertTrue(cadence.due((0,0)),'Stops never wait for the keepalive interval')
        clock.advance(.02);self.assertTrue(cadence.due((-7,0)))
        clock.advance(.099);self.assertFalse(cadence.due((-7,0)))
        clock.advance(.001);self.assertTrue(cadence.due((-7,0)))
        self.assertTrue(cadence.due((-7,0),1),'A newly accepted repeat command is sent immediately')

    def test_display_keeps_last_reading_and_age_but_never_uses_it_for_protection(self):
        clock=Clock();s=ControlState(Config(),clock);s.update_scan(scan())
        clock.advance(1);s.update_scan(None)
        view=s.collision.snapshot()
        self.assertIsNone(view['distances']['front'])
        self.assertEqual(view['last_distances']['front'],1)
        self.assertEqual(view['distance_age_ms']['front'],1000)
        self.assertTrue(s.collision.evaluate(.3,0)[3])

    def test_cached_docker_image_is_pinned_and_unprivileged(self):
        from control_launcher import agent_command
        with patch('control_launcher.shutil.which',return_value='/usr/bin/docker'),patch('control_launcher.os.getuid',return_value=1000,create=True),patch('control_launcher.os.getgid',return_value=1000,create=True),patch('control_launcher.subprocess.run',return_value=Mock(returncode=0,stdout='sha256:cached\n')):
            command,backend=agent_command()
        self.assertEqual(backend,'docker');self.assertIn('sha256:cached',command)
        self.assertNotIn('--privileged',command);self.assertNotIn('pull',command)
        self.assertIn('1000:1000',command);self.assertIn('ROS_DOMAIN_ID=20',command)

    @unittest.skipUnless(importlib.util.find_spec('fcntl'),'Linux runner lock required')
    def test_repeated_agent_crash_has_bounded_retries(self):
        import agent_runner,json
        clock=Clock();child=Mock(returncode=134);child.poll.return_value=134
        with tempfile.TemporaryDirectory() as folder,patch.object(agent_runner,'ROOT',Path(folder)),patch.object(agent_runner,'agent_command',return_value=(['fake-agent'],'native')),patch.object(agent_runner.signal,'signal'),patch.object(agent_runner.subprocess,'Popen',return_value=child) as popen,patch.object(agent_runner.time,'monotonic',side_effect=clock),patch.object(agent_runner.time,'sleep',side_effect=clock.advance):
            agent_runner.main()
            result=json.loads((Path(folder)/'run/agent-status.json').read_text())
            self.assertEqual(result['status'],'FAILED');self.assertEqual(result['last_exit'],134)
            self.assertEqual(popen.call_count,3)


if __name__=='__main__':unittest.main()
