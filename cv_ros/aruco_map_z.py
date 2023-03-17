# ------------------------------------------------------
#
# Тестовая программа для вывода текущей высоты аппарата
# относительно фрейма карты ArUco-меток.
#
# ------------------------------------------------------


import rospy
from clover import srv

rospy.init_node('aruco_map_z')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

while True:
    print(get_telemetry(frame_id="aruco_map").z)
