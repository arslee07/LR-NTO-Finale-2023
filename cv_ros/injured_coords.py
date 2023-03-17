# -----------------------------------------------
#
# Программа для получения координат обнаруженных
# касок (пострадавших) относительно карты ArUco.
# К сожалению, чуда не случилось.
#
# -----------------------------------------------


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

# Размер камеры в пикселях
WIDTH, HEIGHT = 320, 240

# Примерное отношение расстояния к пикселям.
# К сожалению, эта штука плюс-минус точная только
# вдоль середины камеры, так как у камеры есть очень
# сильный эффект рыбьего глаза. =(
SIZE_PIXEL_RATIO = 3 / WIDTH

# HSV-маска цвета касок
INJURED_MASK = ((105, 40, 40), (120, 255, 255))

bridge = CvBridge()

injured_pub = rospy.Publisher("~injured", Image, queue_size=1)


# Метод получения координат центра фигур по заданной цветовой маске.
def get_color_coordinates(img, mask):
    # Координаты центра контура
    def __contour_center(cnt_):
        M = cv2.moments(cnt_)
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    # Фильтр площади контура
    def __enough_area(cnt_):
        return cv2.contourArea(x) > 300

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv_img, (7, 7), 0)
    masked = cv2.inRange(blurred, *mask)
    cnt, _ = cv2.findContours(masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = list(filter(lambda x: cv2.contourArea(x) > 300, cnt))
    return [__contour_center(c) for c in cnt]


while True:
    data = rospy.wait_for_message('main_camera/image_raw', Image)
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    coords = get_color_coordinates(cv_image, INJURED_MASK)
    for c in coords:
        cv2.circle(cv_image, c, 7, (255, 255, 255), -1)
    injured_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    
    if coords:
        c = coords[0]
        telem = get_telemetry(frame_id="aruco_map")
        if not math.isnan(telem.x):
            cx = telem.x + ((c[0] - (WIDTH // 2)) * SIZE_PIXEL_RATIO) if c[0] != 0 else 0
            cy = telem.y + ((c[1] - (HEIGHT // 2)) * SIZE_PIXEL_RATIO) if c[1] != 0 else 0
            print(telem.z, cx, cy)

