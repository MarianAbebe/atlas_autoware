#!/usr/bin/env python3
"""Exercise real relay and completed service callback with a stalled raw reader."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_command_observation import take, relay_module
from command_recording import CommandJournal, payload, recording_barrier
import rclpy
from autoware_control_msgs.msg import Control
from std_srvs.srv import Trigger


class CommandRecording(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node=rclpy.create_node('command_recording_test')

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()

    def test_stalled_observer_recovers_every_delivered_command(self):
        node=self.node
        raw=node.create_publisher(Control,'journal/raw',10)
        output=node.create_publisher(Control,'journal/output',10)
        relay=node.create_subscription(Control,'journal/raw',lambda m:None,10)
        observer=node.create_subscription(Control,'journal/raw',lambda m:None,10)
        delivered=node.create_subscription(Control,'journal/output',lambda m:None,10)
        import time
        until=time.monotonic()+5
        while raw.get_subscription_count()!=2 or output.get_subscription_count()!=1:
            self.assertLess(time.monotonic(),until);rclpy.spin_once(node,timeout_sec=.01)
        journal=CommandJournal()
        state=SimpleNamespace(originals={},inject='live',fault_input='odometry',command_journal=journal)
        rows=[]
        for seq in range(1,31):
            msg=Control();msg.stamp.sec=seq;msg.longitudinal.acceleration=seq/10
            raw.publish(msg)
            incoming,_=take(relay)
            relay_module.Relay.forward(state,'command',incoming,output)
            sent,_=take(delivered)
            self.assertEqual(len(journal.rows),seq,'recording must precede delivery')
            rows.append([0.,*payload(sent)])
        # Observer is deliberately not serviced during delivery. No sleeps drain it.
        node.create_service(Trigger,'journal/snapshot',journal.snapshot)
        client=node.create_client(Trigger,'journal/snapshot')
        result=recording_barrier(node,client,rows)
        self.assertTrue(result['passed'],result['reason'])
        self.assertEqual([row[1:] for row in result['rows']],[row[1:] for row in rows])
        self.assertEqual(len(result['rows']),30)
        print('Journal barrier: 30/30 delivered commands recorded before forwarding; service callback completed')

    def test_missing_delivery_is_not_silently_accepted(self):
        journal=CommandJournal()
        self.node.create_service(Trigger,'missing/snapshot',journal.snapshot)
        client=self.node.create_client(Trigger,'missing/snapshot')
        result=recording_barrier(self.node,client,[[0.,1.,0.,0.,0.]])
        self.assertFalse(result['passed']);self.assertEqual(result['reason'],'delivered_payload_absent')

    def test_overflow_fails_closed_without_interrupting_forwarding(self):
        journal=CommandJournal(capacity=1)
        published=[]
        state=SimpleNamespace(originals={},inject='live',fault_input='odometry',command_journal=journal)
        for seq in range(2):
            msg=Control();msg.stamp.sec=seq
            relay_module.Relay.forward(state,'command',msg,SimpleNamespace(publish=published.append))
        self.assertEqual(len(published),2)
        self.assertEqual(len(journal.rows),1)
        self.node.create_service(Trigger,'overflow/snapshot',journal.snapshot)
        result=recording_barrier(self.node,self.node.create_client(Trigger,'overflow/snapshot'),[[0.,0.,0.,0.,0.]])
        self.assertFalse(result['passed']);self.assertEqual(result['reason'],'journal_overflow')

    def test_command_faults_keep_original_record_and_forwarding_semantics(self):
        for fault in ['drop','freeze']:
            journal=CommandJournal();published=[]
            old=Control();old.stamp.sec=1
            current=Control();current.stamp.sec=2
            state=SimpleNamespace(originals={},inject='live',fault_input='command',
                                  command_journal=journal,fault_snapshot=old)
            publisher=SimpleNamespace(publish=published.append)
            relay_module.Relay.forward(state,'command',old,publisher)
            state.inject=fault
            relay_module.Relay.forward(state,'command',current,publisher)
            self.assertEqual([r[1:] for r in journal.rows],[payload(old),payload(current)])
            self.assertEqual(published,[old] if fault=='drop' else [old,old])
            self.assertEqual(state.originals['command'],current)

    def test_missing_service_is_a_bounded_failure(self):
        result=recording_barrier(self.node,self.node.create_client(Trigger,'absent/snapshot'),[],timeout=.05)
        self.assertFalse(result['passed']);self.assertEqual(result['reason'],'recording_barrier_timeout')

if __name__=='__main__':unittest.main()
