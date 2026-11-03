'''
Test ulog2ros2bag MAVROS ESC topics
'''

import os
import inspect
import unittest
import tempfile

import numpy as np

from pyulog import ulog2ros2bag
from pyulog.core import ULog

TEST_PATH = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
ESC_SAMPLE = os.path.join(TEST_PATH, 'sample_px4_events.ulg')


class TestMavrosEscHelpers(unittest.TestCase):
    """
    Test the ESC tick and unit helpers.
    """

    def test_zero_order_hold_indices(self):
        """
        Each tick takes the latest sample at or before it.
        """
        ts = np.array([1000, 51000, 149000, 151000, 260000], dtype=np.uint64)
        ticks, indices = ulog2ros2bag._zero_order_hold_indices(ts, 100000)
        self.assertEqual(list(ticks), [1000, 101000, 201000])
        self.assertEqual(list(indices), [0, 1, 3])

    def test_esc_temperature_cdegc(self):
        """
        degC -> cdegC, non-finite -> INT16_MAX.
        """
        self.assertEqual(ulog2ros2bag._esc_temperature_cdegc(46.0), 4600)
        self.assertEqual(ulog2ros2bag._esc_temperature_cdegc(-1.5), -150)
        self.assertEqual(ulog2ros2bag._esc_temperature_cdegc(float('nan')), 32767)


@unittest.skipUnless(ulog2ros2bag.ros2_available and ulog2ros2bag.mavros_msgs_available,
                     'ROS2, px4_msgs and mavros_msgs required')
class TestMavrosEscBag(unittest.TestCase):
    """
    Test --mavros-esc end to end against a ULog containing esc_status.
    """

    def test_esc_topics(self):
        """
        Both topics carry one message per 100 ms tick with matching stamps.
        """
        # pylint: disable=import-outside-toplevel, import-error
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from mavros_msgs.msg import ESCInfo, ESCStatus

        esc = ULog(ESC_SAMPLE, ['esc_status']).get_dataset('esc_status')
        ticks, indices = ulog2ros2bag._zero_order_hold_indices(
            esc.data['timestamp'], ulog2ros2bag.ESC_PUBLISH_PERIOD_US)

        with tempfile.TemporaryDirectory() as tmpdir:
            bag = os.path.join(tmpdir, 'bag')
            ulog2ros2bag.convert_ulog2ros2bag(
                ESC_SAMPLE, bag, False, 'esc_status', want_esc=True)

            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('cdr', 'cdr'))
            received = {ulog2ros2bag.ESC_STATUS_TOPIC: [], ulog2ros2bag.ESC_INFO_TOPIC: []}
            types = {ulog2ros2bag.ESC_STATUS_TOPIC: ESCStatus,
                     ulog2ros2bag.ESC_INFO_TOPIC: ESCInfo}
            while reader.has_next():
                topic, raw, t_ns = reader.read_next()
                if topic in received:
                    received[topic].append((t_ns, deserialize_message(raw, types[topic])))

        for topic, msgs in received.items():
            self.assertEqual(len(msgs), len(ticks), topic)
            self.assertEqual([t for t, _ in msgs], [int(t) * 1000 for t in ticks])

        j = indices[-1]
        count = int(esc.data['esc_count'][j])
        status = received[ulog2ros2bag.ESC_STATUS_TOPIC][-1][1]
        info = received[ulog2ros2bag.ESC_INFO_TOPIC][-1][1]
        self.assertEqual(len(status.esc_status), count)
        self.assertEqual(info.count, count)
        self.assertEqual(info.counter, int(esc.data['counter'][j]) & 0xFFFF)
        self.assertEqual(info.info, int(esc.data['esc_online_flags'][j]))
        self.assertEqual(status.esc_status[0].rpm, int(esc.data['esc[0].esc_rpm'][j]))
        self.assertEqual(info.esc_info[0].temperature, ulog2ros2bag._esc_temperature_cdegc(
            float(esc.data['esc[0].esc_temperature'][j])))
