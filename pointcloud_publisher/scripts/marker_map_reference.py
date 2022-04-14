import cv2
import rospy
import rospkg
import tf
import tf2_ros
import geometry_msgs.msg
import warnings
import cv_bridge
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix
from cv2 import aruco

marker_length = 0.14
rot = np.array([[0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 1]],
               dtype=float)
camera_intrinsics = None
image = None
running = True
rospack = rospkg.RosPack()
path = rospack.get_path('pointcloud_publisher')+'/scripts/'


def cameraInfoCallback(camera_info):
    global camera_intrinsics
    camera_intrinsics = camera_info


class image_converter:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/d400/color/image_raw", Image, self.callback)

    def callback(self, data):
        global image
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


def read_dictionary(filename='custom_aruco_dictionary.npy'):
    """Read dictionary from .npy binray file.

    Args:
        filename (str, optional): Name of the ArUco collection. Defaults to 'custom_aruco_dictionary.npy'.

    Returns:
        dictionary: aruco dictionary object.
    """
    # Load dictionary from npy file
    try:
        data = np.load(path+filename, allow_pickle=True, encoding='bytes')
        print("/home/alsarmi/udacity_ws/src/3D-MAV-Path-Planner/pointcloud_publisher/scripts/custom_aruco_dictionary.npy")
        dict = cv2.aruco.custom_dictionary(0, data[0][0])
        dict.maxCorrectionBits = data[0][1]
        dict.bytesList = data[0][2]
    except Exception as e:
        warnings.warn('No such file or directory: ' + filename)
        print(e)
    return dict


def main():
    global running
    rate = rospy.Rate(5)
    broadcaster = tf2_ros.TransformBroadcaster()
    marker_transform = geometry_msgs.msg.TransformStamped()

    marker_transform.header.frame_id = "d400_color_optical_frame"
    marker_transform.child_frame_id = "aruco_detection"

    ic = image_converter()
    #cv2.namedWindow("D435i AruCo", 1)

    dictionary = read_dictionary()
    filtered_translation = None
    filtered_rotation = None
    cnt = 0
    while not rospy.is_shutdown():  # running:
        if image is not None:
            res = cv2.aruco.detectMarkers(image, dictionary)
            if len(res[0]) > 0:
                #cv2.aruco.drawDetectedMarkers(image, res[0], res[1])
                K = np.array(camera_intrinsics.K,
                             dtype=np.float32).reshape(3, 3)
                D = np.array(camera_intrinsics.D, dtype=np.float32)
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    res[0], marker_length, K, D)
                #aruco.drawAxis(image, K, D, rvec, tvec, 0.1)
                rot[:3, :3], _ = cv2.Rodrigues(rvec)
                quaternion = quaternion_from_matrix(rot)
                quaternion_norm = quaternion/np.linalg.norm(quaternion)

                if filtered_translation is None or cnt < 10:
                    filtered_translation = np.array(tvec[0][0])
                if filtered_rotation is None or cnt < 10:
                    filtered_rotation = quaternion_norm

                filtered_rotation = tf.transformations.quaternion_slerp(
                    filtered_rotation, quaternion_norm, 0.01)
                filtered_translation = 0.80*filtered_translation+0.2 * \
                    np.array(tvec[0][0])
                # Detection of marker w.r.t camera optical frame
                marker_transform.transform.translation.x = filtered_translation[0]
                marker_transform.transform.translation.y = filtered_translation[1]
                marker_transform.transform.translation.z = filtered_translation[2]

                marker_transform.transform.rotation.x = filtered_rotation[0]
                marker_transform.transform.rotation.y = filtered_rotation[1]
                marker_transform.transform.rotation.z = filtered_rotation[2]
                marker_transform.transform.rotation.w = filtered_rotation[3]
                marker_transform.header.stamp = rospy.Time.now()

                broadcaster.sendTransform(marker_transform)

                cnt += 1
            # rate.sleep()
        #cv2.imshow("D435i Aruco", image)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
            #running = False


if __name__ == "__main__":

    rospy.init_node('marker_map_reference',
                    anonymous=True, disable_signals=True)
    rospy.Subscriber("/d400/color/camera_info", CameraInfo, cameraInfoCallback)

    main()
