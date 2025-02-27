#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
Control your servo!

---------------------------
Use 'i' to increment angle
Use 'k' to decrement angle
Use 'q' to quit
---------------------------
"""

def getKey():
    """
    Capture a single keypress from stdin (non-blocking).
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)  # save terminal settings

    # 1. Initialize ROS node
    rospy.init_node('servo_key_controller', anonymous=True)

    # 2. Create a publisher on (e.g.) "/servo_angle"
    pub = rospy.Publisher('/servo_angle', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    angle = 90.0  # Start at 90 degrees (neutral for most servos)
    print(msg)

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'i':
                angle += 5
            elif key == 'k':
                angle -= 5
            elif key == 'q':
                print("Quitting...")
                break

            # Clamp angle between [0, 180]
            angle = max(0, min(180, angle))

            # Publish angle
            pub.publish(angle)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Optionally send servo back to neutral
        pub.publish(90)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

