# Управление черепахой по N-угольнику

Этот репозиторий содержит узел ROS-1 `turtle_move.py`, который заставляет черепашку из симулятора `turtlesim` пройти замкнутый путь по правильному N-угольнику. Пользователь вводит длину стороны и количество вершин, после чего программа рассчитывает нужные углы и скорости и последовательно вызывает движение и поворот.

## Установка и запуск
- Зависимости: `ros-noetic` (включая пакеты `turtlesim`, `rospy`, `geometry_msgs.msg`, `turtlesim.msg`).
- Убедитесь, что узел находится в рабочем ROS-пакете (например, `~/catkin_ws/src/polygon_turtle/`), и выполните `catkin_make`.
- Запустите мастера и симулятор:
  1. `roscore`
  2. `rosrun turtlesim turtlesim_node`
- В отдельном терминале выполните узел движения: `rosrun polygon_turtle turtle_move.py`.
- При запуске скрипт запросит ввод параметров: длину стороны (м) и количество вершин (целое число от 3 до 15). Пример: `1.0 6`.

## Пример запуска
```bash
roscore &
rosrun turtlesim turtlesim_node &
rosrun polygon_turtle turtle_move.py
# В консоли узла:
# 1.0 6
```

## Структура кода
- **Глобальное состояние**: переменная `pos` хранит последнюю позу черепахи (см. `turtle_move.py:6`), обновляемую через подписку на `/turtle1/pose`.
- **Функция `pose_callback(msg)`** (см. `turtle_move.py:8`–`turtle_move.py:10`) принимает сообщения `Pose` и обновляет глобальное состояние.
```python
def pose_callback(msg):
    global pos
    pos = msg
```
- **Функция `normalize_angle(angle)`** нормализует любое значение к диапазону `(-π, π]` для корректных ПИД-расчётов (см. `turtle_move.py:12`–`turtle_move.py:13`). Её применение видно в цикле поворота:
```python
    while not rospy.is_shutdown():
        err = normalize_angle(target_angle - pos.theta)
        angle_reached = abs(err) < 0.008
```
- **Функция `move(s, dist)`** отвечает за линейное движение по направлению текущего угла черепахи, контролируя дистанцию и границы поля (см. `turtle_move.py:15`–`turtle_move.py:39`). Она использует `rospy.Rate` и публикует сообщения `Twist` в `/turtle1/cmd_vel`.
```python
    rate = rospy.Rate(50)
    move_cmd = Twist()
    x0, y0 = pos.x, pos.y
    x_aim, y_aim = x0 + cos(pos.theta)*dist, y0 + sin(pos.theta)*dist
    rospy.loginfo("x: %.2f y: %.2f xAim: %.2f yAim: %.2f ", x0, y0, x_aim, y_aim)
    u_min = 0.5
    while not rospy.is_shutdown():
        ux = float(((x_aim-pos.x)**2 + (y_aim-pos.y)**2)**0.5)
        if abs(ux) < 0.008: break
```
- **Функция `turnOnAngle(absolut_angle, u0=0.5)`** реализует ПИД-регулятор угловой скорости, чтобы довести черепаху до заданного абсолютного угла (см. `turtle_move.py:42`–`turtle_move.py:90`). Комбинация `Publisher + Twist.angular.z` позволяет аккуратно ускорять и тормозить при повороте.
```python
    rate = rospy.Rate(rate_hz)
    move_cmd = Twist()
    current_speed = 0.0

    err_old = 0.0
    while not rospy.is_shutdown():
        err = normalize_angle(target_angle - pos.theta)
        angle_reached = abs(err) < 0.008
        derivative = err - err_old
        target_speed = kp * err + kd * derivative
```
- **Главный блок** (см. `turtle_move.py:92`–`turtle_move.py:117`) инициализирует узел, объявляет `Publisher` и `Subscriber`, валидирует ввод и запускает цикл обхода многоугольника.
```python
    while not rospy.is_shutdown():
        for i in range(0, 360, (360//int(N))):
            turnOnAngle(i)
            rospy.sleep(0.250)
            move(0, A)
```

## Работа ROS
- `rospy.init_node('move_turtle', anonymous=True)` (см. `turtle_move.py:93`) регистрирует узел в ROS master.
- `rospy.Subscriber('/turtle1/pose', Pose, pose_callback)` (см. `turtle_move.py:94`) получает обновления положения и передаёт их в callback; связка `Subscriber + callback` обеспечивает синхронизацию состояния.
- `rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)` (см. `turtle_move.py:95`) формирует командный канал скорости. Он работает в паре с сообщением `geometry_msgs.msg.Twist`, где компоненты `linear.x` и `angular.z` задают линейную и угловую скорость.
- `rospy.Rate` используется в функциях `move` и `turnOnAngle` для стабильной частоты публикации команд, а `rospy.sleep` (см. `turtle_move.py:114`–`turtle_move.py:116`) добавляет паузы между действиями, позволяя симулятору применить движение.
- `rospy.loginfo` выводит диагностические сообщения, а `rospy.is_shutdown()` безопасно завершает циклы при остановке узла.
- Топик `/turtle1/pose` (типа `turtlesim/Pose`) даёт актуальное положение; топик `/turtle1/cmd_vel` (типа `geometry_msgs/Twist`) принимает управляющие скорости. Корректное движение достигается благодаря связке «подписчик читает Pose → функции считают ошибку → паблишер отправляет Twist».

## Алгоритм движения
После запуска узла черепашка:
1. Получает параметры длины и количества вершин от пользователя (см. `turtle_move.py:99`–`turtle_move.py:108`).
2. Делит окружность на сектора величиной `360/N`, вычисляя целевые абсолютные углы (см. `turtle_move.py:112`).
3. Для каждой вершины выполняет последовательность `turnOnAngle` → короткая задержка → `move` → задержка (см. `turtle_move.py:112`–`turtle_move.py:116`), пока не обойдёт все грани многоугольника.
4. При каждом шаге `pose_callback` обновляет актуальную позу, `normalize_angle` держит угловую ошибку в пределах `(-π, π]`, а комбинация `Publisher + Twist` обеспечивает передачу команд в `turtlesim`.

Так реализуется замкнутый маршрут: черепашка принимает ввод, вычисляет угол поворота `360/N`, циклически выполняет поворот и движение на заданную длину, пока не вернётся в исходную ориентацию.
