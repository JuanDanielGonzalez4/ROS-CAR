#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

TRIG_PIN = 22  # Change to your GPIO pin
ECHO_PIN = 27  # Change to your GPIO pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def measure_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    timeout = 1.0  # 1-second timeout
    start_wait = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        if time.time() - start_wait > timeout:
            rospy.logwarn("Timeout waiting for echo to start")
            return -1  # Indicate an error

    start_time = time.time()
    start_wait = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()
        if time.time() - start_wait > timeout:
            rospy.logwarn("Timeout waiting for echo to end")
            return -1  # Indicate an error

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance

def ultrasonic_publisher():
    rospy.init_node('ultrasonic_sensor', anonymous=True)
    pub = rospy.Publisher('ultrasonic_distance', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        distance = measure_distance()
        if distance != -1:  # Only publish valid measurements
            rospy.loginfo(f"Distance: {distance:.2f} cm")
            pub.publish(distance)
        rate.sleep()


if __name__ == '__main__':
    try:
        ultrasonic_publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
