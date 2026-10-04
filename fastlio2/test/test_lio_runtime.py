"""Live executable checks. Run inside isolated Humble with --binary/--config."""
import argparse
import os
import signal
import subprocess
import time
import unittest

import rclpy
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rcl_interfaces.srv import GetParameters

parser = argparse.ArgumentParser()
parser.add_argument('--binary', required=True)
parser.add_argument('--config', required=True)
args, unittest_args = parser.parse_known_args()


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('lio_runtime_test')
        self.process = None

    def tearDown(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
                self.fail('worker failed to join within three seconds')
        if self.process and self.process.stdout:
            self.process.stdout.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def start(self, *parameters):
        command = [args.binary, '--ros-args', '-p', 'config_path:=' + args.config,
                   '-r', '__ns:=/runtime_lio']
        for parameter in parameters:
            command += ['-p', parameter]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True)

    def discover(self, publisher):
        deadline = time.monotonic() + 3
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertGreater(publisher.get_subscription_count(), 0)
        # DDS endpoints appear during construction, before spin starts. A
        # service reply proves that the executor can drain a shallow reader.
        client = self.node.create_client(GetParameters, '/runtime_lio/lio_node/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        request = GetParameters.Request()
        request.names = ['lidar_queue_capacity']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2)
        self.assertTrue(future.done(), 'executor did not become ready')
        self.assertIsNotNone(future.result())
        self.node.destroy_client(client)

    def stop(self):
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=3), 0)
        return self.process.communicate()[0]

    @staticmethod
    def imu(nanoseconds):
        message = Imu()
        message.header.stamp.sec = 100
        message.header.stamp.nanosec = nanoseconds
        message.linear_acceleration.z = 0.981
        return message

    @staticmethod
    def scan(nanoseconds):
        message = CustomMsg()
        message.header.stamp.sec = 100
        message.header.stamp.nanosec = nanoseconds
        point = CustomPoint()
        point.x = 2.0
        point.tag = 0x10
        point.offset_time = 50_000_000
        message.points = [point]
        message.point_num = 1
        return message

    def test_rejects_zero_capacity(self):
        self.start('lidar_queue_capacity:=0')
        try:
            code = self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.fail('invalid capacity was accepted')
        self.assertNotEqual(code, 0)

    def test_missing_input_shutdown_joins(self):
        self.start()
        time.sleep(0.4)
        self.assertIsNone(self.process.poll())
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=3), 0)

    def test_missing_imu_interval_exits(self):
        publisher = self.node.create_publisher(Imu, '/livox/imu', 10)
        self.start()
        self.discover(publisher)
        for nanoseconds in (0, 5_000_000, 100_000_000):
            publisher.publish(self.imu(nanoseconds))
            time.sleep(0.03)
        try:
            code = self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.fail('IMU propagation gap was silently accepted')
        self.assertNotEqual(code, 0)
        output = self.process.communicate()[0]
        self.assertIn('gap', output)

    def test_shutdown_joins_while_waiting_for_imu(self):
        publisher = self.node.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.start()
        self.discover(publisher)
        publisher.publish(self.scan(0))
        time.sleep(0.15)
        self.assertIn('scans=1', self.stop())

    def test_invalid_cloud_does_not_consume_imu(self):
        lidar = self.node.create_publisher(CustomMsg, '/livox/lidar', 10)
        imu = self.node.create_publisher(Imu, '/livox/imu', 10)
        self.start()
        self.discover(lidar)
        self.discover(imu)
        imu.publish(self.imu(0))
        for number in range(3):
            scan = self.scan(number * 100_000_000)
            if number == 0:
                scan.points = []
                scan.point_num = 0
            elif number == 1:
                scan.point_num = 2
            else:
                scan.points[0].x = float('nan')
            lidar.publish(scan)
            time.sleep(0.1)
        output = self.stop()
        self.assertIn('invalid=3', output)
        self.assertIn('consumed_imus=0', output)
        self.assertIn('imu_pending=1', output)

    def test_imu_overflow_is_explicit(self):
        publisher = self.node.create_publisher(Imu, '/livox/imu', 10)
        self.start('imu_queue_capacity:=2')
        self.discover(publisher)
        for nanos in (0, 5_000_000, 10_000_000):
            publisher.publish(self.imu(nanos))
            time.sleep(0.03)
        self.assertNotEqual(self.process.wait(timeout=2), 0)
        self.assertIn('IMU buffer overflow', self.process.communicate()[0])

    def test_backward_lidar_is_explicit(self):
        publisher = self.node.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.start()
        self.discover(publisher)
        for nanos in (10_000_000, 0):
            publisher.publish(self.scan(nanos))
            time.sleep(0.03)
        self.assertNotEqual(self.process.wait(timeout=2), 0)
        self.assertIn('backwards', self.process.communicate()[0])

    def test_process_pause_preserves_imu_history(self):
        # Keep the source history too: this test isolates our receiving depth.
        publisher = self.node.create_publisher(Imu, '/livox/imu', 4096)
        self.start()
        self.discover(publisher)
        publisher.publish(self.imu(0))
        time.sleep(0.05)
        self.process.send_signal(signal.SIGSTOP)
        try:
            for number in range(1, 81):
                publisher.publish(self.imu(number * 5_000_000))
                time.sleep(0.005)
        finally:
            self.process.send_signal(signal.SIGCONT)
        for number in range(81, 131):
            publisher.publish(self.imu(number * 5_000_000))
            time.sleep(0.005)
        time.sleep(0.1)
        self.assertIsNone(self.process.poll(), 'continuous source IMU caused a fault after pause')
        output = self.stop()
        self.assertIn(' imus=131 ', output)
        self.assertIn(' duplicate_imus=0 ', output)
        self.assertIn(' consumed_imus=0 ', output)
        self.assertIn(' imu_pending=131 ', output)


if __name__ == '__main__':
    unittest.main(argv=['test_lio_runtime.py'] + unittest_args)
