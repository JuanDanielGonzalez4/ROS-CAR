#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import RPi.GPIO as GPIO

# Change to your desired GPIO pin:
SERVO_PIN = 18  # BCM mode pin 18

# Helper to convert angle [0..180] to duty cycle for SG90 on 50 Hz
def angle_to_duty_cycle(angle):
    # For SG90:
    #  - 0 degrees ~ 2.5% duty
    #  - 180 degrees ~ 12.5% duty
    # That’s a 10% swing across 180 degrees.
    return 2.5 + (angle / 180.0) * 10.0

def angle_callback(msg):
    """
    Subscriber callback for the servo angle topic.
    """
    angle = msg.data
    dc = angle_to_duty_cycle(angle)
    pwm.ChangeDutyCycle(dc)
    rospy.loginfo("Servo angle set to: %.2f (DutyCycle: %.2f)", angle, dc)

def listener():
    """
    Main subscriber setup for /servo_angle.
    """
    rospy.init_node('servo_controller', anonymous=True)

    rospy.Subscriber('/servo_angle', Float64, angle_callback)
    rospy.loginfo("Servo Controller started. Subscribed to /servo_angle.")

    rospy.spin()  # Keep Python from exiting until this node is stopped

if __name__ == '__main__':
    # Set up RPi.GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz for SG90 servo
    pwm.start(0)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()
