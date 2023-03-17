import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped


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


# Поиск очагов возгораний
def fire_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    for coordinate in get_color_coordinates(cv_image, FIRE_MASK):
        cv2.circle(cv_image, coordinate, 7, (255, 255, 255), -1)
    fire_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
fire_sub = rospy.Subscriber("main_camera/image_raw", Image, fire_callback)


# Поиск пострадавших
def injured_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    coords = get_color_coordinates(cv_image, INJURED_MASK)
    for c in coords:
        cv2.circle(cv_image, c, 7, (255, 255, 255), -1)
    injured_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
injred_sub = rospy.Subscriber("main_camera/image_raw", Image, injured_callback)


# Двигаемся по заданной траектории и параллельно ищем очаги и пострадавших
navigate_wait(z=1, frame_id='body', auto_arm=True)
for i in [129, 134, 115, 121, 135, 116, 122, 110, 134, 142, 125]:
    navigate_wait(z=1, frame_id=f'aruco_{i}')
land_wait()

rospy.spin()

