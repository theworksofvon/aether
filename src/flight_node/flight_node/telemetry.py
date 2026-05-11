from geometry_msgs.msg import Vector3
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import String


def nav_sat_status_from_fix_type(fix_type: int) -> int:
    if fix_type >= 3:
        return NavSatStatus.STATUS_FIX
    if fix_type == 2:
        return NavSatStatus.STATUS_SBAS_FIX
    return NavSatStatus.STATUS_NO_FIX


def attitude_message(mavlink_message) -> Vector3:
    message = Vector3()
    message.x = mavlink_message.roll
    message.y = mavlink_message.pitch
    message.z = mavlink_message.yaw
    return message


def gps_message(mavlink_message) -> NavSatFix:
    message = NavSatFix()
    message.latitude = mavlink_message.lat / 1e7
    message.longitude = mavlink_message.lon / 1e7
    message.altitude = mavlink_message.alt / 1000.0
    return message


def gps_status_message(mavlink_message) -> NavSatStatus:
    message = NavSatStatus()
    message.status = nav_sat_status_from_fix_type(mavlink_message.fix_type)
    message.service = NavSatStatus.SERVICE_GPS
    return message


def battery_message(mavlink_message) -> BatteryState:
    message = BatteryState()
    message.voltage = mavlink_message.voltage_battery / 1000.0
    return message


def mode_message(mode_name: str) -> String:
    message = String()
    message.data = mode_name
    return message
