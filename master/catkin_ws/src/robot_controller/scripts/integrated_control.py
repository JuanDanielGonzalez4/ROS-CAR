#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float32
import RPi.GPIO as GPIO
import time

# --- CONFIGURACIÓN DEL SERVO ---
SERVO_PIN = 18  # GPIO donde está conectado el servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz para el SG90
pwm.start(0)

# Variable global para almacenar la última lectura del sensor ultrasónico
current_distance = None

# Umbral en centímetros: si se detecta un objeto a menos de este valor, se bloquea el movimiento
ULTRASONIC_THRESHOLD = 20.0

def angle_to_duty_cycle(angle):
    """Convierte un ángulo [0, 180] en un ciclo de trabajo PWM adecuado."""
    return 2.5 + (angle / 180.0) * 10.0

def ultrasonic_callback(msg):
    global current_distance
    current_distance = msg.data
    rospy.loginfo("Distancia ultrasónica: %.2f cm", current_distance)

def servo_callback(msg):
    global current_distance
    # Si ya tenemos un valor válido y está por debajo del umbral, bloqueamos el movimiento
    if current_distance is not None and current_distance < ULTRASONIC_THRESHOLD:
        rospy.logwarn("Movimiento bloqueado: objeto detectado a %.2f cm", current_distance)
        return

    # Si no hay objeto cercano, permitimos mover el servo
    rospy.loginfo("Movimiento permitido: moviendo el servo a %.2f°", msg.data)
    pwm.ChangeDutyCycle(angle_to_duty_cycle(msg.data))

def main():
    rospy.init_node('integrated_controller', anonymous=True)
    # Suscribirse al tópico de comandos para el servo
    rospy.Subscriber('/servo_angle', Float64, servo_callback)
    # Suscribirse al tópico del sensor ultrasónico
    rospy.Subscriber('ultrasonic_distance', Float32, ultrasonic_callback)
    rospy.loginfo("Nodo integrado con sensor ultrasónico iniciado.")
    rospy.spin()

if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()
