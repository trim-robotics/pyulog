#! /usr/bin/env python

"""
Convert a ULog file into a ROS2 bag.
"""

from collections import defaultdict
import argparse
import math
import re
from os import environ
from pathlib import Path
from importlib.metadata import version
import numpy as np

from .core import ULog

# pylint: disable=too-many-locals, invalid-name, too-many-branches

# Do not exit on failing to import ROS2 packages so they don't block the help message.
ros2_available = True
try:
    from rclpy.serialization import serialize_message  # pylint: disable=import-error
    import rosbag2_py  # pylint: disable=import-error
    from builtin_interfaces.msg import Time  # pylint: disable=import-error
    from geometry_msgs.msg import (  # pylint: disable=import-error
        TransformStamped,
        PoseStamped,
    )
    from tf2_msgs.msg import TFMessage  # pylint: disable=import-error
    from nav_msgs.msg import Odometry, Path  # pylint: disable=import-error
    from sensor_msgs.msg import NavSatFix, NavSatStatus  # pylint: disable=import-error
except ImportError as e:
    print(
        "Error: Could not import ROS2 packages. Make sure you have sourced"
        " your ROS2 installation."
    )
    print("Actual error:", e)
    ros2_available = False

try:
    from px4_msgs import msg as px4_msgs  # pylint: disable=import-error
except ImportError as e:
    print(
        "Error: Could not import px4_msgs. Make sure you have built and sourced"
        " the correct version of px4_msgs."
    )
    print("Actual error:", e)
    ros2_available = False


def main():
    """Command line interface"""

    parser = argparse.ArgumentParser(description="Convert ULog to rosbag")
    parser.add_argument("filename", metavar="file.ulg", help="ULog input file")

    parser.add_argument(
        "-o",
        "--output",
        dest="bag",
        help="rosbag output destination",
        default=None,
    )

    parser.add_argument(
        "--mcap",
        dest="mcap",
        action="store_true",
        help="Use mcap storage in rosbag instead of sqlite3",
        default=False,
    )

    parser.add_argument(
        "-m",
        "--messages",
        dest="messages",
        help=(
            "Only consider given messages. Must be a comma-separated list of"
            " names, like 'sensor_combined,vehicle_gps_position'"
        ),
    )

    parser.add_argument(
        "-i",
        "--ignore",
        dest="ignore",
        action="store_true",
        help="Ignore string parsing exceptions",
        default=False,
    )

    parser.add_argument(
        "-v",
        "--verbose",
        dest="verbose",
        action="store_true",
        help="Print extra debugging information",
        default=False,
    )

    parser.add_argument(
        "--tf",
        dest="tf",
        action="store_true",
        help=(
            "Also synthesize /tf (odom->base_link) and /tf_static (map->odom) from"
            " vehicle_local_position and vehicle_attitude, using MAVROS ENU/FLU"
            " conventions."
        ),
        default=False,
    )

    parser.add_argument(
        "--nav",
        dest="nav",
        action="store_true",
        help=(
            "Also synthesize nav_msgs/Odometry (/odom), nav_msgs/Path (/path) and"
            " sensor_msgs/NavSatFix (/gps/fix)."
        ),
        default=False,
    )

    args = parser.parse_args()

    if not ros2_available:
        return  # Error messages printed in 'except ImportError' blocks above

    convert_ulog2ros2bag(
        args.filename,
        args.bag,
        args.mcap,
        args.messages,
        args.ignore,
        args.verbose,
        args.tf,
        args.nav,
    )


# https://stackoverflow.com/questions/19053707/converting-snake-case-to-lower-camel-case-lowercamelcase
def to_camel_case(snake_str):
    """Convert snake case string to camel case"""
    components = snake_str.split("_")
    return "".join(x.title() for x in components)


def convert_ulog2ros2bag(
    ulog_file_name: str,
    rosbag_name: str | None,
    mcap: bool,
    messages: str,
    disable_str_exceptions=False,
    verbose=False,
    want_tf=False,
    want_nav=False,
):
    """
    Coverts and ULog file to a CSV file.

    :param ulog_file_name: The ULog filename to open and read
    :param rosbag_name: The rosbag filename to open and write
    :param messages: A list of message names
    :param want_tf: Also synthesize /tf and /tf_static
    :param want_nav: Also synthesize Odometry, Path and NavSatFix

    :return: No
    """

    msg_filter = messages.split(",") if messages else None

    # When synthesizing derived topics, make sure their source messages survive
    # an explicit -m/--messages filter.
    if msg_filter is not None and (want_tf or want_nav):
        for required in DERIVED_SOURCE_MESSAGES:
            if required not in msg_filter:
                msg_filter.append(required)

    ulog = ULog(ulog_file_name, msg_filter, disable_str_exceptions)

    multiids: dict[str, set[int]] = defaultdict(set)
    for topic in ulog.data_list:
        multiids[topic.name].add(topic.multi_id)

    # ROS2 boilerplate
    writer = rosbag2_py.SequentialWriter()

    # Default rosbag name from ulg file
    rosbag_name = rosbag_name or "rosbag_" + re.sub(
        r"\.ulg$", "", Path(ulog_file_name).name
    )

    storage_options = rosbag2_py.StorageOptions(
        uri=rosbag_name, storage_id="mcap" if mcap else "sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(storage_options, converter_options)

    # Support rosbag2_py topic sequence number in ROS2 versions greater than Humble
    if rosbag_write_uses_seqnum():

        def rosbag_write(topic, msg, timestamp):
            writer.write(topic, msg, timestamp, 0)

    else:

        def rosbag_write(topic, msg, timestamp):
            writer.write(topic, msg, timestamp)

    # Support rosbag2_py TopicMetadata ID in ROS2 versions greater than Humble
    if rosbag_topicmetadata_uses_id():

        def topic_metadata(name, topic_type):
            return rosbag2_py.TopicMetadata(
                0,
                name=name,  # type: ignore
                type=topic_type,
                serialization_format="cdr",
            )

    else:

        def topic_metadata(name, topic_type):
            return rosbag2_py.TopicMetadata(
                name=name,  # type: ignore
                type=topic_type,
                serialization_format="cdr",
            )

    if verbose:
        print(f"I: ROS2 Distro: {environ.get('ROS_DISTRO') or 'not detected'}")
        px4_msgs_share_dir = None
        try:
            # fmt: off
            from ament_index_python.packages import get_package_share_directory  # pylint: disable=import-error, import-outside-toplevel
            # fmt: on

            px4_msgs_share_dir = get_package_share_directory("px4_msgs")
        except:  # pylint: disable=bare-except
            pass
        print(f"I: ROS2 px4_msgs: {px4_msgs_share_dir or 'not found'}")
        print(
            f"I: PX4 firmware version from ulog: {ulog.get_version_info_str() or 'not found'}"
        )
        print("")

    topic_count, message_count = 0, 0
    for ulg_topic in ulog.data_list:
        topic_message_count = len(ulg_topic.data["timestamp"])

        # Determine ROS2 topic name
        if multiids[ulg_topic.name] == {0}:
            ros2_topic = f"/px4/{ulg_topic.name}"
        else:
            ros2_topic = f"/px4/{ulg_topic.name}_{ulg_topic.multi_id}"

        if verbose:
            ulg_topic_name = ulg_topic.name
            if multiids[ulg_topic.name] != {0}:
                ulg_topic_name += f" ({ulg_topic.multi_id})"
            print(
                f"D: Processing ulog topic {ulg_topic_name} with {topic_message_count}"
                f" messages and ROS2 topic {ros2_topic}"
            )

        # Determine ROS2 message type (px4_msgs)
        msg_type_name = calc_msgtype(ulg_topic.name)
        if msg_type_name is None or not hasattr(px4_msgs, msg_type_name):
            print(
                f"W: Message type '{msg_type_name or to_camel_case(ulg_topic.name)}'"
                f" for {ulg_topic.name} not found in px4_msgs, skipping."
            )
            continue

        MsgType = getattr(px4_msgs, msg_type_name)

        if verbose:
            print(f"D: Found ROS2 message type px4_msgs/msg/{MsgType.__name__}")

        # Check if it is a composed message type
        if any(
            re.match(r"(.*?)\.(.*?)", field.field_name)
            for field in ulg_topic.field_data
        ):
            # TODO: add support for composed message types
            print(f"W: Message type for {ulg_topic.name} is composed, skipping.")
            continue

        # Verify message type
        ros2_fields = MsgType.get_fields_and_field_types().keys()
        px4_fields = [
            re.sub(r"\[\d+\]$", "", data.field_name) for data in ulg_topic.field_data
        ]  # strip array indices
        px4_fields = list(set(px4_fields))  # remove duplicates
        if not all(px4_field in ros2_fields for px4_field in px4_fields):
            print(
                f"W: Message type px4_msgs/msg/{MsgType.__name__} does not match topic"
                f" '{ulg_topic.name}' in ulog, skipping. Please check your version of px4_msgs."
            )
            if verbose:
                for px4_field in [
                    px4_field
                    for px4_field in px4_fields
                    if px4_field not in ros2_fields
                ]:
                    print(
                        f"D: field {px4_field} of {ulg_topic.name} not found in {MsgType.__name__}"
                    )
            continue

        # Register topic in rosbag
        writer.create_topic(
            topic_metadata(ros2_topic, f"px4_msgs/msg/{MsgType.__name__}")
        )

        # Write each message
        for i in range(len(ulg_topic.data["timestamp"])):
            msg = ros2_msg_from_ulg_topic(ulg_topic, i, MsgType)

            ts = ulg_topic.data["timestamp"][i] * 1000  # us -> ns
            rosbag_write(
                ros2_topic,
                serialize_message(msg),
                int(ts),
            )
        topic_count += 1
        message_count += topic_message_count

    if want_tf or want_nav:
        derived_topics, derived_messages = write_derived_topics(
            ulog, writer, topic_metadata, rosbag_write, want_tf, want_nav, verbose
        )
        topic_count += derived_topics
        message_count += derived_messages

    writer.close()
    print(
        f"\nWrote rosbag '{rosbag_name}' with {topic_count} topics and {message_count} messages."
    )


def rosbag_topicmetadata_uses_id():
    """
    rosbag2_py PR #1538 adds a parameter to TopicMetadata.__init__() for a topic ID,
    breaking the interface. This attempts to add compability with both versions.
    """
    return version("rosbag2_py") >= "0.25.0"


def rosbag_write_uses_seqnum():
    """
    rosbag2_py commit 62e5f77 adds a parameter to SequentialWriter.write() for a
    topic sequence number, breaking the interface. This attempts to add compatibility
    with both versions.
    """
    return version("rosbag2_py") >= "0.32.0"


def calc_msgtype(topic_name: str) -> str | None:
    """Calculate message type from topic name, if possible"""

    REGEX_EXCEPTIONS = {
        r"^actuator_controls_status_\d$": "ActuatorControlsStatus",
        r"^actuator_outputs\w*": "ActuatorOutputs",
        r"^config_overrides\w*": "ConfigOverrides",
        r"^estimator_aid_src_((gnss_yaw)|(ev_yaw)|(fake_hgt)|(airspeed)|(sideslip)|(\w+_hgt))$": "EstimatorAidSource1d",  # pylint: disable=line-too-long
        r"^estimator_aid_src_((drag)|(aux_vel)|(optical_flow)|(\w+_pos)|(aux_global_position))$": "EstimatorAidSource2d",  # pylint: disable=line-too-long
        r"^estimator_aid_src_((ev_vel)|(gnss_vel)|(gravity)|(mag))$": "EstimatorAidSource3d",
        r"^estimator_ev_pos_bias": "EstimatorBias3d",
        r"^estimator_((baro)|(gnss_hgt))_bias": "EstimatorBias",
        r"^estimator_innovation\w*": "EstimatorInnovations",
        r"^px4io_status$": "Px4ioStatus",
        r"^vehicle_gps_position$": "SensorGps",
        r"^sensors_status_\w+": "SensorsStatus",
        r"^vehicle_angular_velocity\w*": "VehicleAngularVelocity",
        r"^\w+?attitude(_groundtruth)?$": "VehicleAttitude",
        r"^\w+?attitude_setpoint$": "VehicleAttitudeSetpoint",
        r"^\w+?_command\w*": "VehicleCommand",
        r"^\w+?_control_\w+": "VehicleControlMode",
        r"^((aux_)|(estimator_)|(vehicle_)|(external_ins_))global_position(_groundtruth)?$": "VehicleGlobalPosition",  # pylint: disable=line-too-long
        r"^\w+?_local_position(_groundtruth)?$": "VehicleLocalPosition",
        r"^\w+?_odometry$": "VehicleOdometry",
        r"^((estimator)|(vehicle))_optical_flow_vel$": "VehicleOpticalFlowVel",
        r"^vehicle_thrust_setpoint\w*": "VehicleThrustSetpoint",
        r"^vehicle_torque_setpoint\w*": "VehicleTorqueSetpoint",
        r"^(estimator_)?wind$": "Wind",
    }

    direct_name = to_camel_case(topic_name)
    if hasattr(px4_msgs, direct_name):  # type: ignore
        return direct_name

    result = None
    for pattern, type_name in REGEX_EXCEPTIONS.items():
        if re.match(pattern, topic_name):
            result = type_name
            break

    return result


def ros2ify_value(field_name: str, value, MsgType: object):
    """Translate a pyulog numpy value to ros2 plain python value"""
    value_type: np.signedinteger | np.unsignedinteger | np.floating = value.dtype

    # int8 could either be int8_t, bool, or char
    # check the destination type
    ros2_type: str = MsgType.get_fields_and_field_types()[field_name]  # type: ignore

    # startswith() accounts for array types
    if value_type == np.int8 and ros2_type.startswith("bool"):
        return bool(value)

    return value.item()


def ros2_msg_from_ulg_topic(ulg_topic: ULog.Data, i: int, MsgType):
    """Create a ROS2 message object from a ulg topic message"""
    msg = MsgType()
    for field in ulg_topic.field_data:
        array_match = re.match(r"(.*?)\[(\d+)\]", field.field_name)
        value = ulg_topic.data[field.field_name][i]
        if array_match:
            field_name, array_index = array_match.groups()
            array_index = int(array_index)
            target = getattr(msg, field_name)
            if array_index >= len(target):
                # ULog array is longer than this px4_msgs version's fixed-size
                # array (firmware/px4_msgs version skew); skip the extra elements.
                continue
            value = ros2ify_value(field_name, value, MsgType)
            target[array_index] = value
        else:
            try:
                value = ros2ify_value(field.field_name, value, MsgType)
                setattr(msg, field.field_name, value)
            except Exception as e:
                print(
                    f"Exception when setting field '{field.field_name}' from"
                    f" topic '{ulg_topic.name}' with type: {type(value)}"
                )
                raise e

    return msg


# ---------------------------------------------------------------------------
# Derived ROS topics: /tf, /tf_static, Odometry, Path, NavSatFix
# ---------------------------------------------------------------------------

# ULog messages consumed to synthesize the derived topics.
DERIVED_SOURCE_MESSAGES = [
    "vehicle_local_position",
    "vehicle_attitude",
    "vehicle_global_position",
    "vehicle_gps_position",
]

# Frame names (REP-105).
WORLD_FRAME = "map"
ODOM_FRAME = "odom"
BODY_FRAME = "base_link"

# Publish the accumulated Path at most this often, in log seconds.
PATH_PUBLISH_PERIOD_S = 1.0


def _quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conj(q):
    """Conjugate of a (w, x, y, z) quaternion."""
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_from_rpy(roll, pitch, yaw):
    """Match ftf::quaternion_from_rpy: Rx(roll) * Ry(pitch) * Rz(yaw)."""
    qx = (math.cos(roll / 2.0), math.sin(roll / 2.0), 0.0, 0.0)
    qy = (math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)
    qz = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    return _quat_mul(_quat_mul(qx, qy), qz)


# MAVROS static rotations (see mavros ftf_frame_conversions.cpp).
_NED_ENU_Q = _quat_from_rpy(math.pi, 0.0, math.pi / 2.0)
_AIRCRAFT_BASELINK_Q = _quat_from_rpy(math.pi, 0.0, 0.0)


def _orientation_ned_to_enu(q_ned):
    """PX4 aircraft/FRD-in-NED quaternion -> ROS base_link/FLU-in-ENU."""
    return _quat_mul(_quat_mul(_NED_ENU_Q, q_ned), _AIRCRAFT_BASELINK_Q)


def _vec_ned_to_enu(x, y, z):
    """(North, East, Down) -> (East, North, Up); works for position and velocity."""
    return (y, x, -z)


def _rotate_vector(q, v):
    """Rotate vector v by quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _stamp_from_us(timestamp_us):
    """builtin_interfaces/Time from a ULog microsecond timestamp."""
    ns = int(timestamp_us) * 1000
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _topic_data_by_name(ulog):
    """Map ULog message name -> Data, preferring the multi_id 0 instance."""
    result = {}
    for topic in ulog.data_list:
        if topic.name not in result or topic.multi_id == 0:
            result[topic.name] = topic
    return result


def write_derived_topics(
    ulog, writer, topic_metadata, rosbag_write, want_tf, want_nav, verbose
):
    """Synthesize /tf(+static), Odometry, Path and NavSatFix from PX4 messages.

    Returns (topic_count, message_count) added to the bag.
    """
    data = _topic_data_by_name(ulog)
    topic_count = 0
    message_count = 0

    def create(name, type_str):
        nonlocal topic_count
        writer.create_topic(topic_metadata(name, type_str))
        topic_count += 1

    def write(name, msg, ts_us):
        nonlocal message_count
        rosbag_write(name, serialize_message(msg), int(ts_us) * 1000)  # us -> ns
        message_count += 1

    lp = data.get("vehicle_local_position")
    att = data.get("vehicle_attitude")

    if (want_tf or want_nav) and lp is None:
        print(
            "W: --tf/--nav requested but 'vehicle_local_position' is not in the ULog;"
            " skipping /tf, Odometry and Path."
        )

    if (want_tf or want_nav) and lp is not None:
        lp_ts = lp.data["timestamp"]
        lp_x, lp_y, lp_z = lp.data["x"], lp.data["y"], lp.data["z"]
        lp_vx, lp_vy, lp_vz = lp.data["vx"], lp.data["vy"], lp.data["vz"]

        att_ts = att_q = None
        if att is not None:
            att_ts = att.data["timestamp"]
            att_q = np.stack([att.data[f"q[{k}]"] for k in range(4)], axis=1)

        def orientation_at(ts):
            if att_ts is None or len(att_ts) == 0:
                return (1.0, 0.0, 0.0, 0.0)
            j = int(np.searchsorted(att_ts, ts))
            if j >= len(att_ts):
                j = len(att_ts) - 1
            elif j > 0 and (ts - att_ts[j - 1]) < (att_ts[j] - ts):
                j -= 1
            return _orientation_ned_to_enu(tuple(float(v) for v in att_q[j]))

        if want_tf:
            create("/tf", "tf2_msgs/msg/TFMessage")
            create("/tf_static", "tf2_msgs/msg/TFMessage")
            # map -> odom identity, written once (latched-style) at the first stamp.
            static_tf = TransformStamped()
            static_tf.header.stamp = _stamp_from_us(lp_ts[0])
            static_tf.header.frame_id = WORLD_FRAME
            static_tf.child_frame_id = ODOM_FRAME
            static_tf.transform.rotation.w = 1.0
            write("/tf_static", TFMessage(transforms=[static_tf]), lp_ts[0])

        if want_nav:
            create("/odom", "nav_msgs/msg/Odometry")
            create("/path", "nav_msgs/msg/Path")

        path_poses = []
        last_path_us = None
        path_period_us = PATH_PUBLISH_PERIOD_S * 1e6

        for i in range(len(lp_ts)):
            x, y, z = float(lp_x[i]), float(lp_y[i]), float(lp_z[i])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue  # pre-EKF-init samples are NaN

            ts = lp_ts[i]
            stamp = _stamp_from_us(ts)
            px, py, pz = _vec_ned_to_enu(x, y, z)
            qw, qx, qy, qz = orientation_at(ts)

            if want_tf:
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = ODOM_FRAME
                t.child_frame_id = BODY_FRAME
                t.transform.translation.x = px
                t.transform.translation.y = py
                t.transform.translation.z = pz
                t.transform.rotation.w = qw
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                write("/tf", TFMessage(transforms=[t]), ts)

            if want_nav:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = ODOM_FRAME
                odom.child_frame_id = BODY_FRAME
                odom.pose.pose.position.x = px
                odom.pose.pose.position.y = py
                odom.pose.pose.position.z = pz
                odom.pose.pose.orientation.w = qw
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                # Twist expressed in the child (base_link) frame, per REP-105.
                vx, vy, vz = _vec_ned_to_enu(
                    float(lp_vx[i]), float(lp_vy[i]), float(lp_vz[i])
                )
                bvx, bvy, bvz = _rotate_vector(
                    _quat_conj((qw, qx, qy, qz)), (vx, vy, vz)
                )
                odom.twist.twist.linear.x = bvx
                odom.twist.twist.linear.y = bvy
                odom.twist.twist.linear.z = bvz
                write("/odom", odom, ts)

                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = ODOM_FRAME
                pose.pose = odom.pose.pose
                path_poses.append(pose)

                if last_path_us is None or (ts - last_path_us) >= path_period_us:
                    path = Path()
                    path.header.stamp = stamp
                    path.header.frame_id = ODOM_FRAME
                    path.poses = list(path_poses)
                    write("/path", path, ts)
                    last_path_us = ts

        if want_nav and path_poses:
            # Final full path.
            path = Path()
            path.header.stamp = _stamp_from_us(lp_ts[-1])
            path.header.frame_id = ODOM_FRAME
            path.poses = list(path_poses)
            write("/path", path, lp_ts[-1])

    # ---- NavSatFix from global position (preferred) or raw GPS ----
    if want_nav:
        gp = data.get("vehicle_global_position")
        gps = data.get("vehicle_gps_position")
        if gp is not None:
            src, ts_arr = gp, gp.data["timestamp"]
            lat_arr, lon_arr, alt_arr = gp.data["lat"], gp.data["lon"], gp.data["alt"]
        elif gps is not None:
            src, ts_arr = gps, gps.data["timestamp"]
            lat_arr = gps.data["latitude_deg"]
            lon_arr = gps.data["longitude_deg"]
            alt_arr = gps.data["altitude_msl_m"]
        else:
            src = None
            print("W: --nav requested but no global position / GPS message; skipping NavSatFix.")

        if src is not None:
            create("/gps/fix", "sensor_msgs/msg/NavSatFix")
            for i in range(len(ts_arr)):
                lat, lon, alt = float(lat_arr[i]), float(lon_arr[i]), float(alt_arr[i])
                if not (math.isfinite(lat) and math.isfinite(lon)):
                    continue
                fix = NavSatFix()
                fix.header.stamp = _stamp_from_us(ts_arr[i])
                fix.header.frame_id = BODY_FRAME
                fix.status.status = NavSatStatus.STATUS_FIX
                fix.status.service = NavSatStatus.SERVICE_GPS
                fix.latitude = lat
                fix.longitude = lon
                fix.altitude = alt
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                write("/gps/fix", fix, ts_arr[i])

    if verbose:
        print(f"D: derived topics added: {topic_count}, messages: {message_count}")

    return topic_count, message_count


if __name__ == "__main__":
    main()
