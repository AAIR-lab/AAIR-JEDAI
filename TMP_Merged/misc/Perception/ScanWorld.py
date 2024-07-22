import threading
import rospy
import ar_track_alvar_msgs.msg


import numpy
import openravepy as orpy



class ScanWorld(threading.Thread):

    __instance = None;

    def __init__(self, env, marker_id_object_name_map):
        print "ScanWorld __init__ Called"
        self.env = env.CloneSelf(orpy.CloningOptions.Bodies)
        self.env.SetViewer('RViz')
        self.marker_id_object_name_map = marker_id_object_name_map
        self.detected_object_name_transform_map = {}
        rospy.init_node("scan_world") #init_node must be started in the main thread.
        threading.Thread.__init__(self)

    @staticmethod
    def instance(env=None, marker_id_object_name_map=None):
        if ScanWorld.__instance is None:
            ScanWorld.__instance = ScanWorld(env, marker_id_object_name_map)
            ScanWorld.__instance.start()

        return ScanWorld.__instance

    ##TEST##
    # def get_eye(self):
    #     return self.eye
    ##TEST##

    def get_object_name_transform_map(self):
        return self.detected_object_name_transform_map

    def run(self):
        print "Starting AR Listener: Do make sure that ar_track_alvar has been launched on the Fetch robot!"
        self.marker_subscriber=rospy.Subscriber("ar_pose_marker",ar_track_alvar_msgs.msg.AlvarMarkers, self.onARMarkerTransform)
        rospy.spin()

        ##TEST##
        # i=0
        # while i>=0:
        #     i = i + 1
        #     self.eye = i
        ##TEST##



    def __getPoseObj(self, pos, ori):
        # pdb.set_trace()
        pose_target = geometry_msgs.msg.Pose()
        precision = 3

        pose_target.orientation.x = ori.x
        pose_target.orientation.y = ori.y
        pose_target.orientation.z = ori.z
        pose_target.orientation.w = ori.w
        pose_target.position.x = pos.x
        pose_target.position.y = pos.y
        pose_target.position.z = pos.z

        return pose_target
    
    def __getRaveTransformFromROSPose(self,ros_pose):
        """
        OpenRAVE Convention: Quaternions are defined with the scalar value as the first component. For example [w x y z]
        """
        rave_pose = numpy.asarray([ \
            ros_pose.orientation.w, \
            ros_pose.orientation.x, \
            ros_pose.orientation.y, \
            ros_pose.orientation.z, \
            ros_pose.position.x, \
            ros_pose.position.y, \
            ros_pose.position.z])

        return orpy.matrixFromPose(rave_pose)

    def onARMarkerTransform(self, data):
        body_names = self.env.GetBodies();
        for marker in data.markers:
            print "Marker ID: " + str(marker.id)
            if marker.id not in self.marker_id_object_name_map:
                continue

            pos = marker.pose.pose.position
            ori = marker.pose.pose.orientation
            pose_target = self.__getPoseObj(pos, ori)
            openrave_transform = self.__getRaveTransformFromROSPose(pose_target)
            object_name = self.marker_id_object_name_map.get(marker.id)
            self.detected_object_name_transform_map[object_name] = openrave_transform
            self.env.GetKinBody(object_name).SetTransform(openrave_transform)
