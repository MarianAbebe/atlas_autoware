#!/usr/bin/env python3
import math
import unittest
import json
import array
import tempfile
import threading
from pathlib import Path as FilePath
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from scene import (LIDAR_SERIALIZATION_TIMING_FIELDS, ODOMETRY_TIMING_FIELDS,
    SCAN_DTYPE, SCAN_FIELDS, TICK_TIMING_FIELDS, Scene, SceneTimingHistory,
    packed_xyz_view, source_age_at_receipt_sec, thread_cpu_elapsed_sec, thread_cpu_time_sec,
    transform_world_points, SCAN_FIELDS)
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from run_lap_validation import append_process_event
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from generate_five_lap_map import flatten_route, flatten_segments, loop_segments


class FakePublisher:
    def __init__(self, subscribers=0):
        self.subscribers=subscribers
        self.messages=[]

    def get_subscription_count(self):
        return self.subscribers

    def publish(self,msg):
        self.messages.append(msg)


def fake_scene(truth_stamp=(1,0), path=None):
    scene=SimpleNamespace(
        stopping=False,path=path,truth=SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=truth_stamp[0],nanosec=truth_stamp[1]))),
        initial_sent=False,drop=False,request_engagement=True,
        route_length_requested=16.0,trajectory_repeats=1,loop_transition_max_m=6.5,
        include_terminal_extension=False,map_cloud_topic='',lidar_max_range_m=0.0,
        imu_previous_stamp_ns=None,imu_previous_velocity=None,odom_state_lock=threading.Lock(),
        timing=SceneTimingHistory(),timing_pub=FakePublisher(subscribers=1),
        last_tick_monotonic=None,last_tick_finished_monotonic=None,
        last_scan_publish_monotonic=None,last_odom_callback_monotonic=None,accel_since_tick=0,
        last_accel_header_stamp_ns=None,last_accel_receipt_monotonic=None,
        last_accel_header_age_ros_sec=None,
        process_pid=12345,process_start_monotonic=1.0,process_start_ros_time_ns=1_000_000_000,
        scene_tick_index=0,last_odom_receipt_monotonic=None,last_odom_header_stamp_ns=None,
        last_scan_header_stamp_ns=None,process_resource_sample_monotonic=1.0,
        process_resource_sample=dict(process_cpu_time_sec=0.1,process_cpu_fraction_since_sample=None,
            process_max_rss_kib=1000,system_load_1m=0.1,system_available_memory_bytes=1_000_000),
        odom_callback_count=0,odom_callback_max_sec=0.,odom_callback_gap_sec=None,
        odom_callback_timing_max={field:None for field in ODOMETRY_TIMING_FIELDS},
        route_msg=SimpleNamespace(header=Header()),yaw=0.,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
            nanoseconds=2_000_000_000,to_msg=lambda:Header().stamp)))
    for name in ('init','route','gear','auto_gear','engage','mode','raw','imu'):
        setattr(scene,name,FakePublisher(subscribers=1))
    scene.motion=FakePublisher(subscribers=1)
    scene.record_timing=lambda record,start:Scene.record_timing(scene,record,start)
    scene.publish_tick_state=lambda stamp,record,**kwargs:Scene.publish_tick_state(scene,stamp,record,**kwargs)
    scene.sample_process_resources=lambda now,force=False:None
    return scene


class ScanGeometry(unittest.TestCase):
    def test_distinct_start_finish_loop_repeats_with_connected_transitions(self):
        scene=SimpleNamespace(path=None,lap_path=None,pending_path=None,external_map_world=None,
            external_map_cloud=None,map_cloud_topic='',route_length_requested=sum(
                np.linalg.norm(np.diff(np.asarray(flatten_segments(loop_segments())),axis=0),axis=1)),
            trajectory_repeats=5,include_terminal_extension=True,obstacle=False,timing=SceneTimingHistory(),timing_pub=FakePublisher(),
            loop_transition_max_m=8.5,map_view=FakePublisher(),info=FakePublisher(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=2_000_000_000)))
        scene.record_timing=lambda record,start:Scene.record_timing(scene,record,start)
        msg=Path()
        for x,y in flatten_route():
            pose=PoseStamped();pose.pose.position.x=x;pose.pose.position.y=y
            pose.pose.orientation.w=1.0;msg.poses.append(pose)
        Scene.on_path(scene,msg)
        self.assertAlmostEqual(scene.lap_length_m,63.4,delta=0.3)
        transition_gap=np.linalg.norm(scene.lap_path[0,:2]-scene.lap_path[-1,:2])
        self.assertAlmostEqual(transition_gap,8.0,delta=0.02)
        self.assertGreater(scene.terminal_extension_m,2.0)
        connector_count=math.ceil(transition_gap/0.2)
        expected_length=(len(scene.lap_path)+(4*(connector_count+len(scene.lap_path)-1))+
                         len(scene.terminal_path)-1)
        self.assertEqual(len(scene.path),expected_length)
        self.assertEqual(len(scene.route_msg.points),len(scene.path))
        self.assertEqual(scene.route_msg.points[-1].longitudinal_velocity_mps,0.0)
        self.assertGreater(np.linalg.norm(scene.path[-1,:2]-scene.lap_path[0,:2]),2.0)
        self.assertTrue(all(p.longitudinal_velocity_mps==1.0 for p in scene.route_msg.points[:-1]))
        np.testing.assert_allclose(scene.path[:len(scene.lap_path)],scene.lap_path)
        first_connector=scene.path[len(scene.lap_path):len(scene.lap_path)+connector_count]
        np.testing.assert_allclose(first_connector[0],scene.lap_path[-1]+(scene.lap_path[0]-scene.lap_path[-1])/connector_count)
        np.testing.assert_allclose(scene.path[len(scene.lap_path)+connector_count],scene.lap_path[1])
        yaws=[2*math.atan2(p.pose.orientation.z,p.pose.orientation.w) for p in scene.route_msg.points]
        self.assertGreater(max(yaws)-min(yaws),1.4)

    def test_reused_transform_buffer_matches_inverse_pose(self):
        world=np.array([[10.,20.,3.],[-2.,5.,1.],[0.,0.,0.]])
        original=world.copy();out=np.empty_like(world);scratch=np.empty(len(world))
        yaw=math.pi/4;cx=math.cos(yaw);sx=math.sin(yaw);px,py,pz=4.,-3.,.5
        result=transform_world_points(world,out,scratch,px,py,pz,cx,sx)
        expected=np.column_stack((cx*(world[:,0]-px)+sx*(world[:,1]-py),
            -sx*(world[:,0]-px)+cx*(world[:,1]-py),world[:,2]-pz))
        self.assertIs(result,out);np.testing.assert_allclose(result,expected,rtol=0,atol=1e-12)
        np.testing.assert_array_equal(world,original)

    def test_pointcloud_binary_layout_is_preserved(self):
        self.assertEqual(SCAN_DTYPE.itemsize,26)
        self.assertEqual([SCAN_DTYPE.fields[k][1] for k in ('x','y','z','intensity','tag','line','timestamp')],
            [0,4,8,12,16,17,18])
        expected=[('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),
            ('z',8,PointField.FLOAT32),('intensity',12,PointField.FLOAT32),
            ('tag',16,PointField.UINT8),('line',17,PointField.UINT8),
            ('timestamp',18,PointField.FLOAT64)]
        self.assertEqual([(field.name,field.offset,field.datatype,field.count)
            for field in SCAN_FIELDS],[(name,offset,datatype,1)
            for name,offset,datatype in expected])


class SceneCallbackTiming(unittest.TestCase):
    def test_strided_xyz_assignment_is_byte_identical_to_field_writes(self):
        xyz=np.array([[1.25,-2.5,3.75],[-0.125,4.5,8.0],[9.0,10.25,-1.5]],dtype=np.float64)
        expected=np.zeros(len(xyz),dtype=SCAN_DTYPE);actual=np.zeros(len(xyz),dtype=SCAN_DTYPE)
        expected['intensity']=actual['intensity']=40.
        expected['x']=xyz[:,0];expected['y']=xyz[:,1];expected['z']=xyz[:,2]
        expected['timestamp']=actual['timestamp']=1793417448968673280.
        packed_xyz_view(actual)[:]=xyz
        self.assertEqual(actual.tobytes(),expected.tobytes())
        np.testing.assert_array_equal(actual['x'],expected['x'])
        np.testing.assert_array_equal(actual['y'],expected['y'])
        np.testing.assert_array_equal(actual['z'],expected['z'])

    def test_motion_imu_uses_odometry_stamp_and_turn_kinematics(self):
        scene=fake_scene()
        first=Odometry();first.header.frame_id='hidden_truth_base';first.header.stamp.sec=10
        first.twist.twist.linear.x=.5;first.twist.twist.angular.z=.2
        Scene.on_truth(scene,first)
        second=Odometry();second.header.frame_id='hidden_truth_base';second.header.stamp.sec=10;second.header.stamp.nanosec=100_000_000
        second.twist.twist.linear.x=.6;second.twist.twist.angular.z=.2
        Scene.on_truth(scene,second)
        imu=scene.imu.messages[-1]
        self.assertEqual((imu.header.stamp.sec,imu.header.stamp.nanosec),(10,100_000_000))
        self.assertEqual(imu.header.frame_id,'imu_link')
        self.assertEqual(imu.orientation_covariance[0],-1.0)
        self.assertAlmostEqual(imu.angular_velocity.z,.2)
        self.assertAlmostEqual(imu.linear_acceleration.x,1.0,places=6)
        self.assertAlmostEqual(imu.linear_acceleration.y,.12,places=6)
        self.assertAlmostEqual(imu.linear_acceleration.z,9.80665,places=6)

    def test_stale_odometry_tick_records_each_stage_and_diagnostic_costs(self):
        scene=fake_scene(path=[(0.,0.,0.)])
        Scene.tick(scene)

        self.assertEqual(len(scene.timing.records),1)
        record=scene.timing.records[0]
        self.assertTrue(record['scan_skipped'])
        self.assertEqual(record['skip_reason'],'stale_odometry')
        self.assertTrue(all(field in record for field in LIDAR_SERIALIZATION_TIMING_FIELDS))
        for field in TICK_TIMING_FIELDS:
            self.assertIn(field,record)
            if field != 'diagnostic_publication_sec':
                self.assertIsNotNone(record[field])
                self.assertGreaterEqual(record[field],0.)
        self.assertIsNotNone(record['diagnostic_publication_sec'])
        self.assertGreaterEqual(record['diagnostic_publication_sec'],0.)
        self.assertGreater(record['odometry_freshness_evaluation_sec'],0.)
        self.assertEqual(record['lidar_generation_sec'],0.)
        self.assertEqual(record['lidar_publication_sec'],0.)
        self.assertFalse(record['trajectory_published'])
        self.assertEqual(record['trajectory_publication_sec'],0.)
        self.assertEqual(len(scene.route.messages),0)
        self.assertEqual(len(scene.gear.messages),1)
        self.assertEqual(len(scene.auto_gear.messages),1)
        self.assertEqual(len(scene.engage.messages),1)
        self.assertEqual(len(scene.mode.messages),1)

        streamed=json.loads(scene.timing_pub.messages[0].data)
        self.assertIsNone(streamed['diagnostic_publication_sec'])
        self.assertIsNone(streamed['callback_total_duration_sec'])
        self.assertGreaterEqual(record['diagnostic_recording_sec'],0.)
        self.assertEqual(record['process_pid'],12345)
        self.assertEqual(record['scene_tick_index'],1)
        self.assertIn('ros_time_at_tick_entry_ns',record)
        self.assertIn('last_odom_receipt_age_sec',record)
        self.assertIn('last_scan_publish_age_sec',record)
        self.assertIn('process_max_rss_kib',record)
        self.assertGreaterEqual(record['callback_finish_monotonic_sec'],record['callback_entry_monotonic_sec'])
        self.assertAlmostEqual(record['callback_total_duration_sec'],
            record['callback_finish_monotonic_sec']-record['callback_entry_monotonic_sec'],places=6)
        self.assertIn('process_max_rss_kib',record)

    def test_fresh_scan_is_published_before_route_state(self):
        scene=fake_scene(path=[(0.,0.,0.)])
        truth=Odometry();truth.header.stamp.sec=1;truth.header.stamp.nanosec=950_000_000
        truth.pose.pose.orientation.w=1.0
        scene.truth=truth
        scene.scan_world=np.array([[1.,0.,0.]])
        scene.scan_xyz=np.empty((1,3),dtype=np.float64)
        scene.scan_scratch=np.empty(1,dtype=np.float64)
        scene.scan_points=np.zeros(1,dtype=SCAN_DTYPE)
        scene.scan_xyz_fields=packed_xyz_view(scene.scan_points)
        scene.scan_msg=PointCloud2();scene.scan_msg.point_step=SCAN_DTYPE.itemsize
        scene.scan_msg.fields=list(SCAN_FIELDS);scene.scan_msg.header=Header()
        order=[]
        class OrderedPublisher(FakePublisher):
            def __init__(self,name):super().__init__(subscribers=1);self.name=name
            def publish(self,msg):order.append(self.name);self.messages.append(msg)
        scene.raw=OrderedPublisher('scan');scene.route=OrderedPublisher('route')
        Scene.tick(scene)
        self.assertEqual(order[0:2],['scan','route'])
        self.assertEqual(len(scene.raw.messages),1)
        self.assertEqual(len(scene.route.messages),1)
        self.assertTrue(scene.timing.records[0]['scan_published'])
        self.assertTrue(scene.timing.records[0]['trajectory_published'])
        scan=scene.raw.messages[0]
        self.assertEqual(scan.point_step,SCAN_DTYPE.itemsize)
        self.assertEqual([(f.name,f.offset,f.datatype) for f in scan.fields],
            [(f.name,f.offset,f.datatype) for f in SCAN_FIELDS])

    def test_early_skip_still_has_complete_timing_schema(self):
        scene=fake_scene(path=None)
        Scene.tick(scene)
        record=scene.timing.records[0]
        self.assertEqual(record['skip_reason'],'path_unavailable')
        self.assertTrue(all(field in record for field in TICK_TIMING_FIELDS))
        self.assertEqual(record['trajectory_publication_sec'],0.)
        self.assertIsNone(record['trajectory_publication_cpu_sec'])
        self.assertIsNotNone(record['callback_total_duration_sec'])

    def test_stale_odometry_skip_defers_route_but_keeps_operator_state(self):
        scene=fake_scene(path=[(0.,0.,0.)])
        truth=Odometry();truth.header.stamp.sec=1;truth.header.stamp.nanosec=800_000_000
        truth.pose.pose.orientation.w=1.0
        scene.truth=truth

        Scene.tick(scene)

        record=scene.timing.records[-1]
        self.assertEqual(record['skip_reason'],'stale_odometry')
        self.assertFalse(record['trajectory_published'])
        self.assertEqual(record['trajectory_publication_sec'],0.)
        self.assertIsNone(record['trajectory_publication_cpu_sec'])
        self.assertEqual(scene.route.messages,[])
        self.assertEqual(len(scene.gear.messages),1)
        self.assertEqual(len(scene.auto_gear.messages),1)
        self.assertEqual(len(scene.engage.messages),1)
        self.assertEqual(len(scene.mode.messages),1)

    def test_odometry_callback_preserves_publication_and_resets_maxima_per_tick(self):
        scene=fake_scene(path=[(0.,0.,0.)])
        msg=Odometry();msg.header.frame_id='odom';msg.header.stamp.sec=1;msg.header.stamp.nanosec=800_000_000
        msg.twist.twist.linear.x=1.25;msg.twist.twist.angular.z=.5
        Scene.on_truth(scene,msg)

        self.assertIs(scene.truth,msg)
        self.assertEqual(scene.odom_callback_count,1)
        self.assertIsNotNone(scene.last_odom_receipt_monotonic)
        self.assertEqual(scene.last_odom_header_stamp_ns,1_800_000_000)
        self.assertEqual(len(scene.motion.messages),1)
        published=scene.motion.messages[0]
        self.assertEqual(published.header.frame_id,'base_link')
        self.assertEqual(published.header.stamp,msg.header.stamp)
        self.assertEqual(scene.truth.header.frame_id,'odom')
        self.assertEqual(msg.header.frame_id,'odom')
        self.assertEqual(msg.header.stamp.sec,1)
        self.assertEqual(msg.header.stamp.nanosec,800_000_000)
        self.assertEqual(published.twist.twist.linear.x,1.25)
        self.assertEqual(published.twist.twist.angular.z,.5)
        self.assertEqual(published.twist.covariance[0],.0001)
        self.assertEqual(published.twist.covariance[35],.000025)

        first_max=scene.odom_callback_timing_max.copy()
        Scene.on_truth(scene,msg)
        second_max=scene.odom_callback_timing_max.copy()
        expected_max={field:max(first_max[field],second_max[field])
            for field in ODOMETRY_TIMING_FIELDS}
        self.assertEqual(scene.odom_callback_count,2)
        self.assertEqual(len(scene.motion.messages),2)
        for field in ODOMETRY_TIMING_FIELDS:
            self.assertIsNotNone(scene.odom_callback_timing_max[field])
            self.assertTrue(math.isfinite(scene.odom_callback_timing_max[field]))
        self.assertAlmostEqual(scene.odom_callback_timing_max['odometry_source_age_at_receipt_sec'],.2,places=6)
        self.assertEqual(scene.odom_callback_max_sec,
            scene.odom_callback_timing_max['odometry_callback_total_sec'])


        Scene.tick(scene)
        record=scene.timing.records[-1]
        for field in ODOMETRY_TIMING_FIELDS:
            self.assertEqual(record[field],expected_max[field])
            self.assertIsNotNone(record[field])
            self.assertTrue(math.isfinite(record[field]))
        self.assertEqual(record['odometry_callback_max_duration_sec'],record['odometry_callback_total_sec'])
        self.assertEqual(scene.odom_callback_count,0)
        self.assertEqual(scene.odom_callback_max_sec,0.)
        self.assertTrue(all(value is None for value in scene.odom_callback_timing_max.values()))

        Scene.tick(scene)
        reset_record=scene.timing.records[-1]
        self.assertTrue(all(reset_record[field] is None for field in ODOMETRY_TIMING_FIELDS))

    def test_invalid_or_missing_odometry_stamp_has_no_source_age(self):
        self.assertIsNone(source_age_at_receipt_sec(SimpleNamespace(),2_000_000_000))
        self.assertIsNone(source_age_at_receipt_sec(
            SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=0))),
            2_000_000_000))
        self.assertIsNone(source_age_at_receipt_sec(
            SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=1_000_000_000))),
            2_000_000_000))

        scene=fake_scene(path=[(0.,0.,0.)])
        msg=Odometry()  # A zero/default header stamp is invalid for age measurement.
        Scene.on_truth(scene,msg)
        self.assertEqual(scene.odom_callback_count,1)
        self.assertEqual(len(scene.motion.messages),1)
        self.assertIsNone(scene.odom_callback_timing_max['odometry_source_age_at_receipt_sec'])
        for field in ODOMETRY_TIMING_FIELDS[:-1]:
            self.assertTrue(math.isfinite(scene.odom_callback_timing_max[field]))

    def test_lidar_serialization_timing_fields_and_payload_are_preserved(self):
        scene=fake_scene(path=[(0.,0.,0.)])
        truth=Odometry();truth.header.stamp.sec=1;truth.header.stamp.nanosec=990_000_000
        truth.pose.pose.orientation.w=1.
        scene.truth=truth
        scene.scan_world=np.array([[1.25,-2.5,.5],[-1.,.75,1.]],dtype=np.float64)
        scene.scan_xyz=np.empty_like(scene.scan_world)
        scene.scan_scratch=np.empty(len(scene.scan_world),dtype=np.float64)
        scene.scan_points=np.zeros(len(scene.scan_world),dtype=SCAN_DTYPE)
        scene.scan_points['intensity']=40.
        scene.scan_xyz_fields=packed_xyz_view(scene.scan_points)
        scene.scan_msg=PointCloud2();scene.scan_msg.height=1
        scene.scan_msg.width=len(scene.scan_world);scene.scan_msg.point_step=SCAN_DTYPE.itemsize
        scene.scan_msg.row_step=scene.scan_msg.width*scene.scan_msg.point_step
        scene.scan_msg.fields=SCAN_FIELDS

        Scene.tick(scene)
        record=scene.timing.records[-1]
        self.assertTrue(record['scan_published'])
        self.assertTrue(all(field in record for field in LIDAR_SERIALIZATION_TIMING_FIELDS))
        existing_fields=('pointcloud_field_assignment_sec','pointcloud_field_assignment_cpu_sec','pointcloud_tobytes_sec',
            'pointcloud_data_assignment_sec','pointcloud_serialization_sec',
            'pointcloud_serialization_cpu_sec','pointcloud_tobytes_cpu_sec',
            'pointcloud_data_assignment_cpu_sec','lidar_publication_sec',
            'lidar_publication_cpu_sec','lidar_generation_sec','scan_generation_duration_sec',
            'callback_duration_sec','callback_total_duration_sec',
            'cached_odometry_age_sec','header_age_at_publication_sec')
        self.assertTrue(all(field in record for field in existing_fields))
        for field in ('pointcloud_field_assignment_sec','pointcloud_field_assignment_cpu_sec','pointcloud_tobytes_sec',
            'pointcloud_array_construction_sec','pointcloud_data_assignment_sec','pointcloud_serialization_sec',
            'lidar_generation_sec','lidar_publication_sec','scan_generation_duration_sec'):
            self.assertGreaterEqual(record[field],0.)
        cpu_fields=('pointcloud_field_assignment_cpu_sec','pointcloud_serialization_cpu_sec','pointcloud_tobytes_cpu_sec',
            'pointcloud_array_construction_cpu_sec','pointcloud_data_assignment_cpu_sec',
            'lidar_publication_cpu_sec')
        if record['thread_cpu_timing_supported']:
            for field in cpu_fields:
                self.assertIsNotNone(record[field])
                self.assertTrue(math.isfinite(record[field]))
                self.assertGreaterEqual(record[field],0.)
        else:
            self.assertTrue(any(record[field] is None for field in cpu_fields))

        self.assertIsInstance(scene.scan_msg.data,array.array)
        self.assertEqual(scene.scan_msg.data.typecode,'B')
        self.assertEqual(bytes(scene.scan_msg.data),scene.scan_points.tobytes())
        self.assertEqual(scene.scan_msg.width,len(scene.scan_world))
        self.assertEqual(len(scene.scan_msg.data),scene.scan_msg.width*scene.scan_msg.point_step)
        self.assertEqual([(f.name,f.offset,f.datatype,f.count) for f in scene.scan_msg.fields],
            [(f.name,f.offset,f.datatype,f.count) for f in SCAN_FIELDS])
        np.testing.assert_allclose(scene.scan_points['x'],scene.scan_world[:,0])
        np.testing.assert_allclose(scene.scan_points['y'],scene.scan_world[:,1])
        np.testing.assert_allclose(scene.scan_points['z'],scene.scan_world[:,2])
        acquisition_ns=(truth.header.stamp.sec*1_000_000_000+truth.header.stamp.nanosec)//256*256
        self.assertTrue(np.all(scene.scan_points['timestamp']==float(acquisition_ns)))
        self.assertEqual(record['acquisition_stamp'],[
            acquisition_ns//1_000_000_000,acquisition_ns%1_000_000_000])

    def test_thread_cpu_clock_is_independent_and_unsupported_is_explicit(self):
        with patch('scene.time.thread_time',return_value=5.25) as cpu_clock, \
                patch('scene.time.monotonic',side_effect=AssertionError('wall clock used')):
            self.assertEqual(thread_cpu_time_sec(),5.25)
            self.assertEqual(thread_cpu_elapsed_sec(4.0),1.25)
            self.assertEqual(cpu_clock.call_count,2)
        with patch('scene.time.thread_time',side_effect=NotImplementedError):
            self.assertIsNone(thread_cpu_time_sec())
            self.assertIsNone(thread_cpu_elapsed_sec(4.0))

    def test_pointcloud_data_setter_accepts_byte_payload_variants(self):
        packed=np.zeros(8,dtype=SCAN_DTYPE)
        packed['x']=np.arange(8,dtype=np.float32)
        expected=packed.tobytes()
        payload_view=memoryview(packed).cast('B')

        def make_message():
            msg=PointCloud2();msg.height=1;msg.width=len(packed)
            msg.fields=SCAN_FIELDS;msg.point_step=SCAN_DTYPE.itemsize
            msg.row_step=msg.width*msg.point_step;msg.is_bigendian=False;msg.is_dense=True
            return msg

        explicit=array.array('B',expected)
        reusable=array.array('B',[0])*len(expected)
        memoryview(reusable)[:]=payload_view
        variants=(expected,explicit,payload_view,
            array.array('B',payload_view),reusable)
        for payload in variants:
            with self.subTest(payload_type=type(payload).__name__):
                msg=make_message();msg.data=payload
                self.assertIsInstance(msg.data,array.array)
                self.assertEqual(msg.data.typecode,'B')
                self.assertEqual(bytes(msg.data),expected)
                self.assertEqual(msg.fields,SCAN_FIELDS)
                self.assertEqual(msg.point_step,26)
                self.assertEqual(msg.row_step,len(expected))

        alias=make_message();alias.data=reusable
        self.assertIs(alias.data,reusable)
        original=reusable[0];reusable[0]=original^1
        self.assertEqual(alias.data[0],original^1)
        reusable[0]=original


class ProcessLifecycleInstrumentation(unittest.TestCase):
    def test_validator_lifecycle_events_are_bounded_jsonl_records(self):
        with tempfile.TemporaryDirectory() as directory:
            path=FilePath(directory)/'process_lifecycle.jsonl'
            append_process_event(path,'launch_process_started',process_pid=42,process_group_id=42)
            append_process_event(path,'process_group_signal',signal='SIGINT',process_group_id=42)
            rows=[json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([row['event'] for row in rows],['launch_process_started','process_group_signal'])
        self.assertTrue(all(math.isfinite(row['receipt_monotonic_sec']) for row in rows))
        self.assertEqual(rows[1]['signal'],'SIGINT')


if __name__=='__main__':unittest.main()
