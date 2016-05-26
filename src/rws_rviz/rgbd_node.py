#!/usr/bin/env python

import rospy
from depth_cloud_runner import DepthCloudRunner
from rws_rviz.srv import StartDepthCloud, StartDepthCloudResponse
from rws_rviz.srv import StopDepthCloud, StopDepthCloudResponse

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

def main():
    rospy.init_node('rviz_rgbd_node')
    dc_server = DepthCloudServer()
    start_depth = rospy.Service('start_depth_cloud', StartDepthCloud, dc_server.start)
    stop_depth = rospy.Service('stop_depth_cloud', StopDepthCloud, dc_server.stop)
    rospy.on_shutdown(dc_server.shutdown)
    rospy.spin()

if __name__ == '__main__':
    main()
