from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Подключение к SITL симулятору
time.sleep(5)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Функция для выполнения взлета и перемещения коптера
def takeoff_and_goto(target_location, target_altitude):
    print("Taking off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reached")
            break
        time.sleep(1)

    print("Going to the target location...")
    point = LocationGlobalRelative(*target_location, target_altitude)
    vehicle.simple_goto(point)

    while True:
        distance = get_distance_metres(vehicle.location.global_frame, point)
        print("Distance to target: ", distance)
        if distance <= 1:
            print("Target location reached")
            break
        time.sleep(1)
def get_distance_metres(location1, location2):
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return (dlat**2 + dlong**2)**0.5 * 1.113195e5
# Функция для поворота коптера на заданный азимут
def set_yaw(yaw_angle):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                   # Система и компонент (0 для автопилота)
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Команда поворота
        0,                       # Абсолютный азимут (0 означает относительный)
        0, 0, yaw_angle, 0, 0, 0, 0, 0)      # Параметры команды
    vehicle.send_mavlink(msg)

# Точки А и Б
point_a = [50.450739, 30.461242]
point_b = [50.443326, 30.448078]
altitude = 100  # Высота

# Вызов функций
takeoff_and_goto(point_a, altitude)
set_yaw(350)

# Закрытие соединения
vehicle.close()
