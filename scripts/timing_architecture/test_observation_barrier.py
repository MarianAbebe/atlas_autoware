#!/usr/bin/env python3
"""Actual relay/readers: retain each command before releasing the next step."""
import sys
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_command_observation import take, relay_module
import rclpy
from autoware_control_msgs.msg import Control


class ObservationBarrier(unittest.TestCase):
    def test_acknowledged_step_retains_every_command_at_depth_ten(self):
        rclpy.init();node=rclpy.create_node('timing_observation_barrier')
        try:
            publisher=node.create_publisher(Control,'timing/raw',10)
            output=node.create_publisher(Control,'timing/output',10)
            relay=node.create_subscription(Control,'timing/raw',lambda m:None,10)
            observer=node.create_subscription(Control,'timing/raw',lambda m:None,10)
            delivered=node.create_subscription(Control,'timing/output',lambda m:None,10)
            until=time.monotonic()+5
            while publisher.get_subscription_count()!=2 or output.get_subscription_count()!=1:
                self.assertLess(time.monotonic(),until,'discovery timeout');time.sleep(.01)
            state=SimpleNamespace(originals={},inject='live',fault_input='odometry')
            raw_ids=[];delivered_ids=[]
            for seq in range(1,31):
                msg=Control();msg.stamp.sec=seq;msg.longitudinal.acceleration=.2
                publisher.publish(msg)
                incoming,_=take(relay)
                relay_module.Relay.forward(state,'command',incoming,output)
                sent,_=take(delivered)
                recorded,_=take(observer)  # application-level recording barrier
                self.assertEqual(recorded,sent)
                raw_ids.append(recorded.stamp.sec);delivered_ids.append(sent.stamp.sec)
            self.assertEqual(raw_ids,list(range(1,31)))
            self.assertEqual(delivered_ids,raw_ids)
            print('Observation barrier: 30 published, 30 delivered, 30 recorded, depth=10')
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
