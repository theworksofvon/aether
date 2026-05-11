from std_msgs.msg import Float32MultiArray


def sensor_reading_message(temperature_c: float, humidity_percent: float) -> Float32MultiArray:
    message = Float32MultiArray()
    message.data = [temperature_c, humidity_percent]
    return message
