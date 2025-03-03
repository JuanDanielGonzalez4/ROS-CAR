import time

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

# Use BCM numbering for GPIO pins
GPIO.setmode(GPIO.BCM)

# Define GPIO pins (modify based on your wiring)
PWM_PIN_L = 18  # PWM signal to the PmodHB5
DIR_PIN_L = 23  # Optional: Direction control pin
PWM_PIN_R = 13
DIR_PIN_R = 24

# Set up the GPIO pins
GPIO.setup(PWM_PIN_L, GPIO.OUT)
GPIO.setup(DIR_PIN_L, GPIO.OUT)
GPIO.setup(PWM_PIN_R, GPIO.OUT)
GPIO.setup(DIR_PIN_R, GPIO.OUT)

# Set the initial direction (e.g., forward)]
GPIO.output(DIR_PIN_L, GPIO.HIGH)
GPIO.output(DIR_PIN_R, GPIO.LOW)

# Create a PWM instance on PWM_PIN at 1 kHz frequency
pwm_l = GPIO.PWM(PWM_PIN_L, 2000)
pwm_l.start(0)  # Start with 0% duty cycle
pwm_r = GPIO.PWM(PWM_PIN_R, 2000)
pwm_r.start(0)  # Start with 0% duty cycle


def move_left_motor(speed: float):
    speed = max(min(speed, 100), -100)
    print(f"Moving LEFT Motor - Speed: {speed}")
    if speed > 0:
        time.sleep(0.05)
        GPIO.output(DIR_PIN_L, GPIO.HIGH)
    elif speed < 0:
        GPIO.output(DIR_PIN_L, GPIO.LOW)
    else:
        pwm_l.ChangeDutyCycle(0)

    pwm_l.ChangeDutyCycle(abs(speed))


def move_right_motor(speed: float):
    print(f"Moving Right Motor - Speed: {speed}")
    speed = max(min(speed, 100), -100)
    if speed > 0:
        time.sleep(0.05)
        GPIO.output(DIR_PIN_R, GPIO.LOW)

    elif speed < 0:
        time.sleep(0.05)
        GPIO.output(DIR_PIN_R, GPIO.HIGH)

    else:
        pwm_r.ChangeDutyCycle(0)

    pwm_r.ChangeDutyCycle(abs(speed))


def cmd_vel_callback(msg):
    linear_vel = msg.linear.x  # Velocidad lineal (m/s)
    angular_vel = msg.angular.z

    wheel_distance = 0.175  # Distancia entre ruedas en metros
    max_speed = 100  # Máximo valor de PWM (0-100)
    scale_factor = 100

    left_speed = (linear_vel - (angular_vel * wheel_distance / 2)) * scale_factor
    right_speed = (linear_vel + (angular_vel * wheel_distance / 2)) * scale_factor
    # Normalizar velocidades para que no excedan max_speed
    max_calculated = max(abs(left_speed), abs(right_speed))
    if max_calculated > max_speed:
        left_speed = (left_speed / max_calculated) * max_speed
        right_speed = (right_speed / max_calculated) * max_speed

    # Enviar velocidades a los motores
    move_left_motor(left_speed)
    move_right_motor(right_speed)


if __name__ == "__main__":
    try:
        rospy.init_node("robot_controller_pmodhb5", anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
        rospy.spin()  # Mantener el nodo activo
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
