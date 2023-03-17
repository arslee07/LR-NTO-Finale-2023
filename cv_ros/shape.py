# ---------------------------------------
#
# Тестовая программа для вывода размера
# изображения с камеры.
#
# ---------------------------------------


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("recognition")
bridge = CvBridge()

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    print(cv_image.shape)


sub = rospy.Subscriber("main_camera/image_raw", Image, callback)

rospy.spin()

