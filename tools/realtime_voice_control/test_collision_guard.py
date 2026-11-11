"""Collision thresholds, direction interlocks, expiry and actual isolated ROS output."""
import importlib.util
import json
import math
import struct
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
import uuid

from collision_guard import CollisionConfig, CollisionGuard
from dual_control_voice_server import Config, ControlState, PublisherThread


class Clock:
    def __init__(self): self.now=100.
    def __call__(self): return self.now
    def advance(self, seconds): self.now+=seconds


def scan(front=1., left=1., rear=1., right=1.):
    return SimpleNamespace(range_min=.05,range_max=12.,angle_min=0.,
        angle_increment=math.pi/2,ranges=[front,left,rear,right],
        header=SimpleNamespace(frame_id='laser_frame'))


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()
        self.guard=CollisionGuard(clock=self.clock)

    def test_invalid_parameters_are_rejected(self):
        for kwargs in (dict(stop=.4),dict(warning=float('nan')),dict(scan_timeout=0),
                       dict(linear_limit=2),dict(hysteresis=.1),dict(side_warning=float('nan')),
                       dict(side_stop=.2),dict(side_slowdown=.11)):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                CollisionConfig(**kwargs)

    def test_startup_and_invalid_ranges_never_allow_motion(self):
        self.assertTrue(self.guard.evaluate(.5,0)[3])
        for invalid in (float('nan'),float('inf'),0.,-1.):
            self.guard.update(scan(front=invalid))
            self.assertTrue(self.guard.evaluate(.5,0)[3])
        json.dumps(self.guard.snapshot(),allow_nan=False)

    def test_one_close_point_stops_without_requiring_eleven_points(self):
        self.guard.update(scan(front=.25))
        self.assertEqual(self.guard.evaluate(.5,0)[:2],(0,0))
        self.assertEqual(self.guard.action,'STOP')

    def test_distance_boundaries_and_release_hysteresis(self):
        self.guard.update(scan(front=.45))
        self.assertEqual(self.guard.levels['front'],'WARNING')
        self.guard.update(scan(front=.30))
        self.assertEqual(self.guard.levels['front'],'SLOW')
        self.guard.update(scan(front=.25))
        self.assertEqual(self.guard.levels['front'],'STOP')
        self.guard.update(scan(front=.28))
        self.assertEqual(self.guard.levels['front'],'STOP')
        self.guard.update(scan(front=.281))
        self.assertEqual(self.guard.levels['front'],'SLOW')
        self.guard.update(scan(front=.331))
        self.assertEqual(self.guard.levels['front'],'WARNING')
        self.guard.update(scan(front=.481))
        self.assertEqual(self.guard.levels['front'],'CLEAR')

    def test_warning_does_not_change_input(self):
        self.guard.update(scan(front=.40))
        self.assertEqual(self.guard.evaluate(.35,0),(.35,0,1.,False))
        self.assertEqual(self.guard.status,'WARNING')

    def test_side_boundaries_and_hysteresis_are_independent_of_front_rear(self):
        for distance, expected in ((.225,'WARNING'),(.15,'SLOW'),(.10,'STOP'),
                                   (.13,'STOP'),(.131,'SLOW'),(.181,'WARNING'),(.256,'CLEAR')):
            self.guard.update(scan(front=.25,rear=.25,left=distance,right=distance))
            for zone in ('left','right'):
                self.assertEqual(self.guard.levels[zone],expected,(zone,distance))
            self.assertEqual(self.guard.levels['front'],'STOP')
            self.assertEqual(self.guard.levels['rear'],'STOP')

    def test_float32_side_stop_boundary_is_not_rounded_out_of_stop_zone(self):
        distance=struct.unpack('f',struct.pack('f',.1))[0]
        self.assertGreater(distance,.1)
        self.guard.update(scan(left=distance))
        self.assertTrue(self.guard.evaluate(0,.3)[3])
        self.assertEqual(self.guard.levels['left'],'STOP')
        self.assertIn('10cm',self.guard.message)

    def test_side_obstacles_do_not_block_pure_forward_or_reverse(self):
        self.guard.update(scan(left=.08,right=.08))
        for speed in (.35,-.35):
            self.assertEqual(self.guard.evaluate(speed,0),(speed,0,1.,False))
        self.assertTrue(self.guard.evaluate(0,.3)[3])

    def test_turning_uses_reduced_side_thresholds(self):
        self.guard.update(scan(left=.2,right=.2))
        self.assertEqual(self.guard.evaluate(.35,.4),(.35,.4,1.,False))
        self.assertEqual(self.guard.status,'WARNING')
        self.guard.update(scan(left=.14))
        result=self.guard.evaluate(.35,.4)
        self.assertFalse(result[3])
        self.assertEqual(self.guard.action,'LIMIT')
        self.assertAlmostEqual(result[1],.15)
        self.guard.update(scan(left=.1))
        self.assertTrue(self.guard.evaluate(.35,.4)[3])

    def test_mixed_blocking_message_reports_each_directions_own_stop_distance(self):
        self.guard.update(scan(front=.2,left=.08))
        self.guard.evaluate(0,.3)
        self.assertIn('前方处于25cm',self.guard.message)
        self.assertIn('左侧处于10cm',self.guard.message)

    def test_slowdown_preserves_sign_and_never_increases_tiny_input(self):
        self.guard.update(scan(front=.28,rear=.28))
        self.assertAlmostEqual(self.guard.evaluate(.5,0)[0],.15)
        self.assertAlmostEqual(self.guard.evaluate(-.5,0)[0],-.15)
        self.assertEqual(self.guard.evaluate(.05,0)[0],.05)

    def test_front_obstacle_allows_a_fresh_rearward_escape(self):
        self.guard.update(scan(front=.2))
        self.assertTrue(self.guard.evaluate(.3,0)[3])
        self.assertEqual(self.guard.evaluate(-.3,0)[:2],(-.3,0))
        self.assertFalse(self.guard.awaiting_command)
        self.assertTrue(self.guard.evaluate(0,.3)[3])

    def test_rotation_checks_every_side(self):
        for zone in ('front','rear','left','right'):
            self.guard.update(scan(**{zone:.08}))
            self.assertTrue(self.guard.evaluate(0,.3)[3])
            self.assertTrue(self.guard.evaluate(0,-.3)[3])

    def test_missing_rear_rays_dont_prove_reverse_is_safe(self):
        self.guard.update(scan(rear=float('inf')))
        self.assertFalse(self.guard.evaluate(.2,0)[3])
        self.assertTrue(self.guard.evaluate(-.2,0)[3])

    def test_scan_expiry_stops_even_without_a_new_scan_callback(self):
        self.guard.update(scan())
        self.clock.advance(.501)
        self.assertTrue(self.guard.evaluate(.3,0)[3])
        self.assertEqual(self.guard.status,'NO_DATA')

    def test_idle_stop_message_tracks_scan_expiry_instead_of_old_obstacle(self):
        self.guard.update(scan(front=.2))
        self.guard.evaluate(.3,0)
        self.assertIn('停车区',self.guard.message)
        self.clock.advance(.501)
        self.guard.evaluate(0,0)
        self.assertEqual(self.guard.status,'NO_DATA')
        self.assertIn('雷达扫描过期',self.guard.message)
        self.assertNotIn('停车区',self.guard.message)

    def test_idle_stop_message_tracks_obstacle_after_scan_recovers(self):
        self.guard.evaluate(.3,0)
        self.guard.update(scan(front=.2))
        self.guard.evaluate(0,0)
        self.assertEqual(self.guard.status,'STOP')
        self.assertIn('停车区',self.guard.message)
        self.assertNotIn('过期',self.guard.message)
        self.guard.update(scan(front=float('nan')))
        self.guard.evaluate(0,0)
        self.assertEqual(self.guard.status,'NO_DATA')
        self.assertIn('前方无有效测距',self.guard.message)

    def test_bad_geometry_is_a_fault_not_an_exception(self):
        for value in (0,float('nan'),float('inf')):
            msg=scan()
            msg.angle_increment=value
            self.guard.update(msg)
            self.assertTrue(self.guard.evaluate(.2,0)[3])

    def test_invalid_scan_does_not_erase_stop_release_hysteresis(self):
        self.guard.update(scan(front=.25))
        self.guard.update(scan(front=float('nan')))
        self.guard.update(scan(front=.26))
        self.assertEqual(self.guard.levels['front'],'STOP')

    def test_alarm_event_is_deduplicated_and_survives_clearance(self):
        self.guard.update(scan(front=.2))
        self.guard.evaluate(0,0)
        nearby=self.guard.event_id
        self.guard.evaluate(.3,0)
        stopped=self.guard.event_id
        self.assertGreater(stopped,nearby)
        for _ in range(5):
            self.guard.evaluate(.3,0)
            self.guard.evaluate(0,0)
        self.assertEqual(self.guard.event_id,stopped)
        self.guard.update(scan())
        self.guard.evaluate(0,0)
        self.assertTrue(self.guard.awaiting_command)
        self.assertEqual(self.guard.status,'WAIT_COMMAND')
        self.assertEqual(self.guard.last_event['status'],'STOP')

    def test_configured_front_angle_is_applied_to_sector_geometry(self):
        guard=CollisionGuard(CollisionConfig(front_degrees=90),self.clock)
        guard.update(scan(left=.2))
        self.assertEqual(guard.distances['front'],.2)
        self.assertTrue(guard.evaluate(.3,0)[3])

    def test_explicit_disabled_mode_is_reported(self):
        guard=CollisionGuard(CollisionConfig(enabled=False),self.clock)
        self.assertEqual(guard.evaluate(.5,.5),(.5,.5,1.,False))
        self.assertEqual(guard.snapshot()['status'],'DISABLED')


class StateProtectionTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()
        self.state=ControlState(Config(),self.clock)
        self.state.connect('a')
        self.state.claim('a','FULL')
        self.state.update_scan(scan())

    def speak(self,cid,intent):
        if not self.state.clients[cid]['mic']:
            self.state.attach_mic(cid,cid)
        self.state.audio_received(cid,cid)
        epoch,_=self.state.generation(cid)
        return self.state.speech_result(cid,cid,intent,True,intent,self.clock(),epoch)

    def test_enabled_by_default(self):
        self.assertTrue(Config().collision_enabled)

    def test_stop_clears_touch_input_and_stale_http_epoch(self):
        self.state.axis_command('a',.5,0)
        epoch,_=self.state.generation('a')
        self.state.update_scan(scan(front=.2))
        self.assertEqual(self.state.output(),(0,0))
        self.assertEqual(self.state.clients['a']['linear'],0)
        with self.assertRaises(ValueError):
            self.state.axis_command('a',.5,0,epoch)

    def test_scan_clearance_and_reset_never_restore_old_motion(self):
        self.state.axis_command('a',.5,0)
        self.state.update_scan(scan(front=.2))
        self.state.update_scan(scan())
        self.assertEqual(self.state.output(),(0,0))
        self.state.reset('a')
        self.assertEqual(self.state.output(),(0,0))
        self.state.axis_command('a',.25,0)
        self.assertEqual(self.state.output(),(5,0))

    def test_voice_forward_is_blocked_but_new_backward_is_accepted(self):
        self.state.update_scan(scan(front=.2))
        self.assertFalse(self.speak('a','forward'))
        self.assertEqual(self.state.output(),(0,0))
        self.assertIn('防撞',self.state.snapshot('a')['voice_feedback'])
        self.assertTrue(self.speak('a','backward'))
        self.assertEqual(self.state.output(),(-7,0))

    def test_old_voice_result_cannot_replay_after_collision_stop(self):
        self.speak('a','forward')
        epoch,_=self.state.generation('a')
        self.state.update_scan(scan(front=.2))
        self.state.update_scan(scan())
        self.assertFalse(self.state.speech_result('a','a','前进',True,'forward',self.clock(),epoch))
        self.assertEqual(self.state.output(),(0,0))

    def test_slowdown_persists_until_operator_explicitly_changes_input(self):
        self.state.axis_command('a',.5,0)
        self.state.update_scan(scan(front=.28))
        self.assertAlmostEqual(self.state.output()[0],3)
        self.state.update_scan(scan())
        self.assertAlmostEqual(self.state.output()[0],3)
        self.state.axis_command('a',.5,0)
        self.assertEqual(self.state.output(),(10,0))

    def test_dual_role_collision_stop_clears_both_phones(self):
        self.state.release_role('a')
        self.state.claim('a','LINEAR')
        self.state.connect('b')
        self.state.claim('b','ANGULAR')
        self.state.axis_command('a',.35,0)
        self.state.axis_command('b',0,.4)
        self.state.update_scan(scan(left=.1))
        self.assertEqual(self.state.output(),(0,0))
        self.assertEqual(self.state.clients['a']['linear'],0)
        self.assertEqual(self.state.clients['b']['angular'],0)

    def test_dual_roles_move_past_twenty_centimetre_side_clearance(self):
        self.state.release_role('a')
        self.state.claim('a','LINEAR')
        self.state.join('b','ANGULAR')
        self.state.update_scan(scan(left=.2,right=.2))
        self.state.axis_command('a',.35,0)
        self.state.axis_command('b',0,.4)
        self.assertEqual(self.state.output(),(7,120))
        self.assertEqual(self.state.clients['a']['linear'],.35)
        self.assertEqual(self.state.clients['b']['angular'],.4)

    def test_configured_side_distances_reach_filter_and_phone_snapshot(self):
        state=ControlState(Config(collision_side_warning=.21,collision_side_slowdown=.14,
                                  collision_side_stop=.09),self.clock)
        state.update_scan(scan(left=.095))
        snapshot=state.snapshot('')['collision']
        self.assertEqual(snapshot['levels']['left'],'SLOW')
        self.assertEqual(snapshot['thresholds_by_direction']['left'],
                         dict(warning=.21,slowdown=.14,stop=.09))
        self.assertEqual(snapshot['thresholds_by_direction']['front'],
                         dict(warning=.45,slowdown=.30,stop=.25))

    def test_legacy_scaling_preserves_an_idle_peer_slot(self):
        self.state.release_role('a')
        self.state.claim('a','LEFT')
        self.state.join('b','RIGHT')
        self.state.command('a',.4)
        self.state.update_scan(scan(front=.28))
        self.assertEqual(self.state.clients['b']['value'],0)
        self.assertLessEqual(abs(self.state.output()[0]),3+1e-8)
        self.assertLessEqual(abs(self.state.output()[1]),45+1e-8)

    def test_manual_estop_remains_stronger_than_escape_permission(self):
        self.state.update_scan(scan(front=.2))
        self.state.stop('a',latch=True)
        with self.assertRaises(ValueError):
            self.state.axis_command('a',-.3,0)
        self.state.reset('a')
        self.state.axis_command('a',.3,0)
        self.assertEqual(self.state.output(),(0,0))

    def test_only_affected_direction_is_blocked(self):
        self.state.update_scan(scan(rear=.2))
        self.state.axis_command('a',.3,0)
        self.assertEqual(self.state.output(),(6,0))
        self.state.axis_command('a',-.3,0)
        self.assertEqual(self.state.output(),(0,0))

    def test_stale_scan_clears_motion_without_changing_estop_latch(self):
        self.state.axis_command('a',.3,0)
        self.clock.advance(.501)
        self.assertEqual(self.state.output(),(0,0))
        self.assertFalse(self.state.estop)
        self.assertTrue(self.state.snapshot('a')['collision']['awaiting_new_command'])

    def test_invalid_scan_safely_reaches_the_final_output_filter(self):
        self.state.axis_command('a',.3,0)
        self.state.update_scan(SimpleNamespace())
        self.assertEqual(self.state.output(),(0,0))
        json.dumps(self.state.snapshot('a'),allow_nan=False)


@unittest.skipUnless(importlib.util.find_spec('rclpy'),'source ROS2 for the isolated collision publisher test')
class CollisionRosTests(unittest.TestCase):
    def test_private_scan_drives_private_output_interlocks(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import LaserScan
        prefix='/voice_collision_test_'+uuid.uuid4().hex
        context=Context()
        rclpy.init(context=context)
        node=Node('collision_test_observer',context=context)
        executor=SingleThreadedExecutor(context=context)
        executor.add_node(node)
        samples=[]
        node.create_subscription(Twist,prefix,lambda m:samples.append((m.linear.x,m.angular.z)),10)
        pub=node.create_publisher(LaserScan,prefix+'_scan',10)
        current=LaserScan()
        current.range_min=.05;current.range_max=12.
        current.angle_min=0.;current.angle_max=3*math.pi/2
        current.angle_increment=math.pi/2
        current.ranges=[1.,1.,1.,1.]
        scan_enabled=[True]
        node.create_timer(.05,lambda:pub.publish(current) if scan_enabled[0] else None)
        state=ControlState(Config(cmd_topic=prefix,scan_topic=prefix+'_scan',imu_topic=prefix+'_imu'))
        state.connect('test');state.claim('test','FULL')
        publisher=PublisherThread(state)
        def until(predicate,timeout=3):
            deadline=time.monotonic()+timeout
            while not predicate() and time.monotonic()<deadline:
                state.heartbeat('test')
                executor.spin_once(timeout_sec=.02)
                if publisher.error:raise publisher.error
            self.assertTrue(predicate(),(samples[-8:],state.snapshot('test')))
        def expect(value):
            samples.clear()
            until(lambda:any(abs(v-value)<1e-6 and abs(w)<1e-6 for v,w in samples))
        try:
            publisher.start()
            until(lambda:state.collision.levels['front']=='CLEAR')
            state.axis_command('test',.5,0);expect(10)
            current.ranges=[.28,1.,1.,1.];expect(3)
            current.ranges=[.20,1.,1.,1.];expect(0)
            current.ranges=[1.,1.,1.,1.]
            until(lambda:state.collision.levels['front']=='CLEAR')
            self.assertEqual(state.output(),(0,0))
            state.axis_command('test',-.3,0);expect(-6)
            scan_enabled[0]=False
            expect(0)
            self.assertTrue(state.collision.awaiting_command)
            state.set_escape_mode('test',True,state.generation('test')[0])
            state.axis_command('test',.3,0);expect(6)
            state.axis_command('test',0,.5);samples.clear()
            until(lambda:any(abs(v)<1e-6 and abs(w-150)<1e-6 for v,w in samples))
            self.assertFalse(state.collision.fresh(),'No scan is available during this test')
            state.set_escape_mode('test',False,state.generation('test')[0]);expect(0)
        finally:
            publisher.close()
            executor.shutdown(timeout_sec=1)
            node.destroy_node()
            context.shutdown()


if __name__=='__main__':
    unittest.main(verbosity=2)
