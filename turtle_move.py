import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

pos = Pose()

def pose_callback(msg):
    global pos
    pos = msg

def normalize_angle(angle):
    return (angle + pi) % (2 * pi) - pi

def move(s, dist):
    global pos

    rate = rospy.Rate(50)
    move_cmd = Twist()

    x0, y0 = pos.x, pos.y
    x_aim, y_aim = x0 + cos(pos.theta)*dist, y0 + sin(pos.theta)*dist
    rospy.loginfo("x: %.2f y: %.2f xAim: %.2f yAim: %.2f ", x0, y0, x_aim, y_aim)
    u_min = 0.5
    while not rospy.is_shutdown():
        ux = float(((x_aim-pos.x)**2 + (y_aim-pos.y)**2)**0.5)
        if abs(ux) < 0.008: break
        if pos.y > 10.5 or pos.y < 0.5 or pos.x > 10.5 or pos.x < 0.5:
            rospy.loginfo("Черепашка в шаге до вылета за границу ;)")
            rospy.loginfo("Программа остановлена")
            move_cmd.linear.x = 0.0
            pub.publish(move_cmd)
            exit()
            while True: pass
        move_cmd.linear.x = ux + 0.5*(ux//abs(ux))
        pub.publish(move_cmd)
    move_cmd.linear.x = 0.0
    move_cmd.linear.y = 0.0
    pub.publish(move_cmd)


def turnOnAngle(absolut_angle, u0=0.5):
    global pos

    # u0 = 1.2*(absolut_angle/120)

    target_angle = absolut_angle / 180.0 * pi
    kp, kd = 0.4, 1
    max_speed = 2.0
    min_speed = min(abs(u0), max_speed)
    max_accel = 3.0
    angle_deadband = 0.02
    rate_hz = 50.0
    dt = 1.0 / rate_hz

    rate = rospy.Rate(rate_hz)
    move_cmd = Twist()
    current_speed = 0.0

    err_old = 0.0
    while not rospy.is_shutdown():
        err = normalize_angle(target_angle - pos.theta)
        angle_reached = abs(err) < 0.008

        derivative = err - err_old
        target_speed = kp * err + kd * derivative
        target_speed = max(-max_speed, min(max_speed, target_speed))

        if abs(target_speed) < min_speed and abs(err) > angle_deadband:
            direction = 1 if (target_speed if target_speed != 0 else err) >= 0 else -1
            target_speed = min_speed * direction

        delta_speed = target_speed - current_speed
        max_delta = max_accel * dt
        if abs(delta_speed) > max_delta:
            delta_speed = max_delta * (1 if delta_speed > 0 else -1)

        current_speed += delta_speed
        move_cmd.angular.z = current_speed
        pub.publish(move_cmd)

        err_old = err

        if angle_reached and abs(current_speed) < 0.02:
            break

        rate.sleep()

    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    rospy.init_node('move_turtle', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1.0)

    while True:
        f = 0
        A, N = map(float, input("Запишите длину стороны и кол-во углов через пробел: ").split(" "))
        print(A, N)
        if 0.5 <= A and A <= 2.0:
            f += 1
        else: rospy.loginfo("Длина стороны многоугольника не принадлежит промежутку от 0.5 до 2.0")
        if (3 <= N and N <= 15) and (N == int(N)):
            f += 1
        else: rospy.loginfo("количество углов многоугольника N не принадлежит промежутку от 3 до 15, где N принадлежит N")
        if f == 2:
            break

    while not rospy.is_shutdown():
        for i in range(0, 360, (360//int(N))):
            turnOnAngle(i)
            rospy.sleep(0.250)
            move(0, A)
            rospy.sleep(0.250)
        break
