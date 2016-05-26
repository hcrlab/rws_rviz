import rospy

class PointCloudRunner(object):
    def __init__(self):
        self._encoder_clients = {}
        self._encoder_processes = {}
