#!/usr/bin/env python

import rospy
from depth_cloud_runner import DepthCloudRunner
from point_cloud_runner import PointCloudRunner
from rws_rviz.srv import StartDepthCloud, StartDepthCloudResponse
from rws_rviz.srv import StopDepthCloud, StopDepthCloudResponse
from rws_rviz.srv import StartPointCloud, StartPointCloudResponse
from rws_rviz.srv import StopPointCloud, StopPointCloudResponse


class DepthCloudServer(object):
    def __init__(self):
        self._dc = DepthCloudRunner()

    def start(self, req):
        name = self._dc.run(req.depth_topic, req.rgb_topic, req.client_id)
        return StartDepthCloudResponse(name)

    def stop(self, req):
        self._dc.stop(req.depth_topic, req.rgb_topic, req.client_id)
        return StopDepthCloudResponse()

    def shutdown(self):
        self._dc.shutdown()


class PointCloudServer(object):
    def __init__(self):
        self._pc = PointCloudRunner()

    def start(self, req):
        name = self._pc.run(req.topic, req.camera_frame_id, req.client_id)
        return StartPointCloudResponse(name)

    def stop(self, req):
        self._pc.stop(req.topic, req.client_id)
        return StopPointCloudResponse()

    def shutdown(self):
        self._pc.shutdown()


def main():
    rospy.init_node('rviz_rgbd_node')
    dc_server = DepthCloudServer()
    start_depth = rospy.Service('start_depth_cloud', StartDepthCloud,
                                dc_server.start)
    stop_depth = rospy.Service('stop_depth_cloud', StopDepthCloud,
                               dc_server.stop)
    pc_server = PointCloudServer()
    start_pc = rospy.Service('start_point_cloud', StartPointCloud,
                             pc_server.start)
    stop_pc = rospy.Service('stop_point_cloud', StopPointCloud, pc_server.stop)

    def shutdown():
        dc_server.shutdown()
        pc_server.shutdown()

    rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
