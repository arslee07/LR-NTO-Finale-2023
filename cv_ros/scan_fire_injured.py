# ------------------------------------------------------
#
# Программа для полета по заданной траектории и
# поиску возгораний и пострадавших.
# Должна была использоваться в 3 день зачета, но
# произошел небольшой прикол и торжества не состоялось.
#
# ------------------------------------------------------


import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
import math


rospy.init_node("recognition")

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# HSV маски
FIRE_MASK = ((10, 40, 40), (24, 255, 255))
INJURED_MASK = ((105, 40, 40), (120, 255, 255))

bridge = CvBridge()


# Топики с распознаванием
fire_pub = rospy.Publisher("~fire", Image, queue_size=1)
injured_pub = rospy.Publisher("~injured", Image, queue_size=1)

# Полет до определенной точки относительно фрейма
def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)

# Посадка
def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

# Получаем кординаты центров фигур по цветовой маске
def get_color_coordinates(img, mask):
    def __contour_center(cnt_):
        M = cv2.moments(cnt_)
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv_img, (7, 7), 0)
    masked = cv2.inRange(blurred, *mask)
    cnt, _ = cv2.findContours(masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = list(filter(lambda x: cv2.contourArea(x) > 300, cnt))
    return [__contour_center(c) for c in cnt]


visited = set()
def scan_point(point):
    def __image_center_dist(p):
        return math.sqrt((p[0] - (WIDTH // 2)) ** 2 + (p[1] - (HEIGHT // 2) ** 2))

    navigate_wait(z=1, frame_id=f"aruco_{point}")

    telem = get_telemetry(frame_id="aruco_map")

    data = rospy.wait_for_message("main_camera/image_raw", Image)
    img = bridge.imgmsg_to_cv2(data, "bgr8")

    print(f"Coordinates: {telem.x} {telem.y}")
    coords = get_color_coordinates(img, FIRE_MASK)
    if not coords:
        print("No flames found")
    else:
        print(f"Found {len(coords)} flames")
        for c in coords:
            cv2.circle(img, c, 7, (255, 255, 255), -1)
        fire_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

    coords = get_color_coordinates(img, INJURED_MASK)
    if not coords:
        print("No injured found")
    else:
        print(f"Found {len(coords)} injured")
        for c in coords:
            cv2.circle(img, c, 7, (255, 255, 255), -1)
        injured_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    print()


# Двигаемся по заданной траектории и параллельно ищем очаги и пострадавших
navigate_wait(z=1, frame_id='body', auto_arm=True)
for i in [129, 134, 115, 121, 153, 116, 122, 110, 134, 142, 125]:
    scan_point(i)
land_wait()

rospy.spin()
