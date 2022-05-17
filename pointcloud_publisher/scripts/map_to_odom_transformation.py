import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg


def main():
    """
    Calculates the transform from map to odom frame given a detection of a custom ARuCo marker.
    """
    map_to_odom_stable = False
    rate = rospy.Rate(5)
    filtered_map_2_odom_translation = None
    filtered_map_2_odom_rotation = None
    cnt = 0
    map_2_odom = geometry_msgs.msg.TransformStamped()
    odom_2_marker = geometry_msgs.msg.TransformStamped()
    map_2_marker = geometry_msgs.msg.TransformStamped()

    map_2_marker = buff.lookup_transform(
        'map', 'aruco',
        rospy.Time(0),
        rospy.Duration(1.0))
    while not map_to_odom_stable:
        # ----------------------------------------------------------------------
        if not buff.can_transform('t265_odom_frame', 'aruco_detection', rospy.Time.now(), rospy.Duration(1.0)):
            # Calculate the transform from to map to odom
            rospy.logwarn_throttle(
                5.0, 'No tranform from aruco_detection to t265_odom_frame')
        else:
            try:
                odom_2_marker = buff.lookup_transform(
                    't265_odom_frame', 'aruco_detection',
                    rospy.Time(0),
                    rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(
                    5.0, 'No tranform from aruco_detection to t265_odom_frame')
                rate.sleep()
                continue
            T_Odom_Marker = tf_conversions.fromTf(
                ((odom_2_marker.transform.translation.x,
                  odom_2_marker.transform.translation.y,
                  odom_2_marker.transform.translation.z),
                 (odom_2_marker.transform.rotation.x,
                  odom_2_marker.transform.rotation.y,
                  odom_2_marker.transform.rotation.z,
                  odom_2_marker.transform.rotation.w)))
            T_Map_Marker = tf_conversions.fromTf(
                ((map_2_marker.transform.translation.x,
                  map_2_marker.transform.translation.y,
                  map_2_marker.transform.translation.z),
                 (map_2_marker.transform.rotation.x,
                  map_2_marker.transform.rotation.y,
                  map_2_marker.transform.rotation.z,
                  map_2_marker.transform.rotation.w)))
            T_Map_Odom = T_Map_Marker * T_Odom_Marker.Inverse()
            map_2_odom.header.frame_id = "map"
            map_2_odom.child_frame_id = "t265_odom_frame"
            ((map_2_odom.transform.translation.x,
              map_2_odom.transform.translation.y,
              map_2_odom.transform.translation.z),
             (map_2_odom.transform.rotation.x,
              map_2_odom.transform.rotation.y,
              map_2_odom.transform.rotation.z,
              map_2_odom.transform.rotation.w)) = tf_conversions.toTf(T_Map_Odom)

            # Send Transform after certain time to stabilize detection
            if cnt == 100:
                map_to_odom_stable = True
            cnt += 1
    while not rospy.is_shutdown():
        map_2_odom.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(map_2_odom)


# A way to run the code only if the file is run directly.
if __name__ == "__main__":
    rospy.init_node('map_to_odom_transformation',
                    anonymous=True, disable_signals=True)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    buff = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buff)

    main()