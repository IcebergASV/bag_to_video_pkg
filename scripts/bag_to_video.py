#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class BagToVideo:
    def __init__(self):
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 20.0, (640,480))
        rospy.loginfo("BagToVideo initialized, subscribing to /camera/image_raw")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("Image received and converted")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        self.out.write(cv_image)
        rospy.loginfo("Image written to video")
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

def main():
    rospy.init_node('bag_to_video', anonymous=True)
    bag_to_video = BagToVideo()
    rospy.loginfo("bag_to_video_node started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down bag_to_video_node")
    finally:
        cv2.destroyAllWindows()
        bag_to_video.out.release()
        rospy.loginfo("Video writer released")

if __name__ == '__main__':
    main()
