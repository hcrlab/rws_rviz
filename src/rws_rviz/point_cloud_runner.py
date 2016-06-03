import os
import rospy
import subprocess


class PointCloudRunner(object):
    def __init__(self):
        self._encoder_clients = {}
        self._encoder_processes = {}

    def run(self, topic, camera_frame_id, client_id):
        topic = rospy.names.resolve_name_without_node_name(topic)
        name = '/' + '{}'.format(topic).replace('/', '_')[1:]
        name = name.replace('_raw', '_r_aw')
        if topic in self._encoder_clients:
            self._encoder_clients[topic].append(client_id)
        else:
            self._encoder_clients[topic] = [client_id]
            node_name = 'depthcloud_encoder__{}'.format(name[1:])
            encoder_args = ['rosrun', 'depthcloud_encoder',
                            'depthcloud_encoder_node',
                            '__name:={}'.format(node_name),
                            '_cloud:={}'.format(topic), '_depth_source:=cloud',
                            '_camera_frame_id:={}'.format(camera_frame_id),
                            'depthcloud_encoded:={}'.format(name)]
            print 'Launching encoder for client {}: {}'.format(
                client_id, ' '.join(encoder_args))
            self._encoder_processes[topic] = subprocess.Popen(encoder_args,
                                                              env=os.environ)
        return name

    def stop(self, topic, client_id):
        topic = rospy.names.resolve_name_without_node_name(topic)
        if topic in self._encoder_clients:
            try:
                self._encoder_clients[topic].remove(client_id)
            except ValueError:
                pass
            if len(self._encoder_clients[topic]) == 0:
                del self._encoder_clients[topic]
                if topic in self._encoder_processes:
                    print 'Shutting down encoder for {}'.format(topic)
                    self._encoder_processes[topic].kill()
                    del self._encoder_processes[topic]

    def shutdown(self):
        for topic, process in self._encoder_processes.items():
            print 'Shutting down encoder for {}'.format(topic)
            process.kill()
        self._encoder_clients.clear()
        self._encoder_processes.clear()
