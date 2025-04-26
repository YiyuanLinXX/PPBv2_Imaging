#!/usr/bin/env python3
# coding=utf-8
# =============================================================================
# multi_camera_trigger_node.py
# ROS2 node: use single hardware trigger to drive multiple cameras concurrently,
# continuously save raw BayerRG8 images as PGM until interrupted,
# record camera chunk timestamps, ROS timestamps, and synchronized GPS coords into per-camera CSVs.
# Configure exposure, white balance, and gain at initialization.
# =============================================================================

import os
import time
import datetime
import PySpin
import serial as pyserial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from collections import deque

class MultiCameraTriggerNode(Node):
    def __init__(self):
        super().__init__('multi_camera_trigger')
        # Declare parameters
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('arduino_baud', 9600)
        self.declare_parameter('exposure_time', 20000.0)
        self.declare_parameter('gain', 5.0)
        self.declare_parameter('wb_red', 1.34)
        self.declare_parameter('wb_blue', 2.98)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('gps_qos_depth', 50)

        # Read parameters
        p = self.get_parameter
        self.output_dir = p('output_dir').get_parameter_value().string_value
        self.arduino_port = p('arduino_port').get_parameter_value().string_value
        self.arduino_baud = p('arduino_baud').get_parameter_value().integer_value
        self.exposure_time = p('exposure_time').get_parameter_value().double_value
        self.gain_value = p('gain').get_parameter_value().double_value
        self.wb_red = p('wb_red').get_parameter_value().double_value
        self.wb_blue = p('wb_blue').get_parameter_value().double_value
        self.gps_topic = p('gps_topic').get_parameter_value().string_value
        qdepth = p('gps_qos_depth').get_parameter_value().integer_value

        # Prepare GPS subscription with history cache
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qdepth
        )
        self.gps_history = deque(maxlen=qdepth)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_callback, qos)

        # Prepare output directory
        os.makedirs(self.output_dir, exist_ok=True)
        # Open Arduino serial and send start
        try:
            self.arduino = pyserial.Serial(self.arduino_port, self.arduino_baud, timeout=1)
            self.arduino.write(b's')  # start trigger
            self.get_logger().info(f"Sent 's' to Arduino on {self.arduino_port}")
        except Exception as e:
            self.get_logger().error(f"Error opening serial port {self.arduino_port}: {e}")
            rclpy.shutdown()
            return

        # Initialize camera system
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        if self.cam_list.GetSize() == 0:
            self.get_logger().error('No cameras detected. Exiting.')
            self.cleanup()
            rclpy.shutdown()
            return

        # Configure each camera
        self.cameras = []
        for i in range(self.cam_list.GetSize()):
            cam = self.cam_list[i]
            cam.Init()
            nodemap = cam.GetNodeMap()

            # Configure DeviceLinkThroughputLimit
            dltl_node = PySpin.CIntegerPtr(nodemap.GetNode('DeviceLinkThroughputLimit'))
            if PySpin.IsAvailable(dltl_node) and PySpin.IsWritable(dltl_node):
                # Example: limit to 100MB/s
                dltl_node.SetValue(100_000_000)
                self.get_logger().info(f'Set Device Link Throughput Limit to {dltl_node.GetValue()} Bytes/sec')
            else:
                self.get_logger().warn('DeviceLinkThroughputLimit not available or not writable.')

            # Configure exposure
            exp_node = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime'))
            if PySpin.IsWritable(exp_node):
                exp_node.SetValue(self.exposure_time)
                self.get_logger().info(f'Set ExposureTime to {exp_node.GetValue()} µs')
            # Configure gain
            ga = PySpin.CEnumerationPtr(nodemap.GetNode('GainAuto'))
            if PySpin.IsWritable(ga):
                ga.SetIntValue(ga.GetEntryByName('Off').GetValue())
                gn = PySpin.CFloatPtr(nodemap.GetNode('Gain'))
                if PySpin.IsWritable(gn):
                    gn.SetValue(min(gn.GetMax(), self.gain_value))
                    self.get_logger().info(f'Set Gain to {gn.GetValue()} dB')
            # Configure white balance
            wb = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceWhiteAuto'))
            if PySpin.IsWritable(wb):
                wb.SetIntValue(wb.GetEntryByName('Off').GetValue())
                sel = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceRatioSelector'))
                for name, val in [('Red', self.wb_red), ('Blue', self.wb_blue)]:
                    e = sel.GetEntryByName(name); sel.SetIntValue(e.GetValue())
                    br = PySpin.CFloatPtr(nodemap.GetNode('BalanceRatio')); br.SetValue(val)
                self.get_logger().info(f'Set WB ratios: Red={self.wb_red}, Blue={self.wb_blue}')
            # Configure Bayer format
            pf = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            pf.SetIntValue(pf.GetEntryByName('BayerRG8').GetValue())
            # Configure chunk data
            cm = PySpin.CBooleanPtr(nodemap.GetNode('ChunkModeActive')); cm.SetValue(True)
            cs = PySpin.CEnumerationPtr(nodemap.GetNode('ChunkSelector'))
            for name in ['FrameID', 'Timestamp']:
                cs.SetIntValue(cs.GetEntryByName(name).GetValue())
                ce = PySpin.CBooleanPtr(nodemap.GetNode('ChunkEnable')); ce.SetValue(True)
            # Configure hardware trigger
            tm = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerMode'))
            tm.SetIntValue(tm.GetEntryByName('Off').GetValue())
            ts = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSelector'))
            ts.SetIntValue(ts.GetEntryByName('FrameStart').GetValue())
            src = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource'))
            src.SetIntValue(src.GetEntryByName('Line0').GetValue())
            tm.SetIntValue(tm.GetEntryByName('On').GetValue())
            cam.BeginAcquisition()
            # Prepare storage and CSV
            tl = cam.GetTLDeviceNodeMap()
            sn_node = PySpin.CStringPtr(tl.GetNode('DeviceSerialNumber'))
            serial = sn_node.GetValue() if PySpin.IsReadable(sn_node) else f'cam{i}'
            cam_dir = os.path.join(self.output_dir, serial)
            os.makedirs(cam_dir, exist_ok=True)
            csv_path = os.path.join(cam_dir, 'Timestamp_GPS.csv')
            csv_file = open(csv_path, 'w', newline='')
            header = 'Frame ID,Computer Time,ROS Time Stamp(s.ns),Chunk Frame ID,Chunk Time,Latitude,Longitude,FixQuality\n'
            csv_file.write(header)
            csv_file.flush()
            self.cameras.append({
                'cam': cam,
                'nodemap': nodemap,
                'dir': cam_dir,
                'serial': serial,
                'csv': csv_file,
                'counter': 1
            })

    def _gps_callback(self, msg: NavSatFix):
        t = msg.header.stamp
        ts = t.sec + t.nanosec * 1e-9
        self.gps_history.append((ts, msg.latitude, msg.longitude, msg.status.status))

    def run(self):
        self.get_logger().info('Starting acquisition loop...')
        try:
            while rclpy.ok():
                # process incoming GPS callbacks
                rclpy.spin_once(self, timeout_sec=0)
                for entry in self.cameras:
                    img = entry['cam'].GetNextImage(2000)
                    if not img.IsIncomplete():
                        # record start time
                        t0 = time.perf_counter()
                        comp_str = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]
                        t = self.get_clock().now().to_msg()
                        ros_ts = f"{t.sec}.{t.nanosec:09d}"
                        ros_float = t.sec + t.nanosec * 1e-9
                        cd = img.GetChunkData()
                        cfid = cd.GetFrameID()
                        cts = cd.GetTimestamp()
                        # sync GPS: pick closest
                        lat = ''; lon = ''; fq = ''
                        if self.gps_history:
                            best = min(self.gps_history, key=lambda x: abs(x[0] - ros_float))
                            _, lat, lon, fq = best
                        # save image
                        w, h = img.GetWidth(), img.GetHeight()
                        data = img.GetData()
                        fn = os.path.join(entry['dir'], f"{entry['counter']:06d}.pgm")                        
                        with open(fn, 'wb') as f:
                            # Write proper PGM header
                            f.write(b'P5\n')
                            # Width and height
                            f.write(f'{w} {h}\n'.encode())
                            # Max grayscale value
                            f.write(b'255\n')
                            # Image data
                            f.write(data)

                        # record end time
                        t1 = time.perf_counter()
                        elapsed = t1 - t0
                        self.get_logger().info(
                            f"camera {entry['serial']} frame {entry['counter']:06d} saved in {elapsed:.3f}s"
                        )
                        # write CSV
                        line = (
                            f"{entry['counter']},{comp_str},{ros_ts},"
                            f"{cfid},{cts:.5E},{lat},{lon},{fq}\n"
                        )
                        entry['csv'].write(line)
                        entry['csv'].flush()
                        entry['counter'] += 1
                    img.Release()
        except KeyboardInterrupt:
            self.get_logger().info('Stopping acquisition')
        finally:
            self.cleanup()

    def cleanup(self):
        try:
            self.arduino.write(b'e')
            self.arduino.close()
        except:
            pass
        for entry in self.cameras:
            try:
                entry['cam'].EndAcquisition()
                tm = PySpin.CEnumerationPtr(entry['nodemap'].GetNode('TriggerMode'))
                tm.SetIntValue(tm.GetEntryByName('Off').GetValue())
                entry['cam'].DeInit()
            except:
                pass
            try:
                entry['csv'].close()
            except:
                pass
        try:
            self.cam_list.Clear()
            self.system.ReleaseInstance()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraTriggerNode()
    if rclpy.ok():
        node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
