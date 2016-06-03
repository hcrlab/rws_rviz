"""Runs the backend ROS nodes necessary for depth clouds.

Depth clouds are point clouds synthesized from depth + color images.
Clients can ask to have a depth cloud visualized by specifying a depth image
topic, a color image topic, and a unique client ID. For each depth topic, we
run the depth_image_proc/convert_metric nodelet and pipe the output to the
depthcloud_encoder. Although its feasible to replicate the functionality of
these two nodes in C++, we just launch these as processes in the shell for
simplicity.
"""
import os
import rospy
import subprocess


class DepthCloudRunner(object):
    def __init__(self):
        # Maps depth topics to IDs of clients who are using them.
        self._converter_clients = {}

        # Maps depth topics to depth_image_proc/convert_metric nodelets that
        # are running.
        self._converter_processes = {}

        # Maps (depth, rgb) topics to IDs of clients who are using them.
        self._encoder_clients = {}

        # Maps (depth, rgb) topics to depthcloud_encoder processes that are
        # running.
        self._encoder_processes = {}

    def run(self, depth_topic, rgb_topic, client_id):
        depth_topic = rospy.names.resolve_name_without_node_name(depth_topic)
        rgb_topic = rospy.names.resolve_name_without_node_name(rgb_topic)
        converted_topic = rospy.names.resolve_name_without_node_name(
            '/float/{}'.format(depth_topic))

        if depth_topic in self._converter_clients:
            self._converter_clients[depth_topic].append(client_id)
        else:
            self._converter_clients[depth_topic] = [client_id]
            converter_args = ['rosrun', 'nodelet', 'nodelet', 'standalone',
                              'depth_image_proc/convert_metric',
                              'image_raw:={}'.format(depth_topic),
                              'image:={}'.format(converted_topic)]
            print 'Launching converter for client {}: {}'.format(
                client_id, ' '.join(converter_args))
            self._converter_processes[depth_topic] = subprocess.Popen(
                converter_args,
                env=os.environ)

        key = (depth_topic, rgb_topic)
        name = '/' + '{}_{}'.format(depth_topic, rgb_topic).replace('/', '_')[1:]
        # ros_web_video ignores all topics with '_raw' in them, so we escape
        # all instances of '_raw'
        # TODO(jstn): Once web_video_server works with point clouds, switch to
        # web_video_server and remove this escaping.
        name = name.replace('_raw', '_r_aw')
        node_name = 'depthcloud_encoder__{}'.format(name[1:])
        if key in self._encoder_clients:
            self._encoder_clients[key].append(client_id)
        else:
            self._encoder_clients[key] = [client_id]
            encoder_args = ['rosrun', 'depthcloud_encoder',
                            'depthcloud_encoder_node',
                            '__name:={}'.format(node_name),
                            '_rgb:={}'.format(rgb_topic),
                            '_depth:={}'.format(converted_topic),
                            'depthcloud_encoded:={}'.format(name)]
            print 'Launching encoder for client {}: {}'.format(
                client_id, ' '.join(encoder_args))
            self._encoder_processes[key] = subprocess.Popen(encoder_args,
                                                            env=os.environ)
        return name

    def stop(self, depth_topic, rgb_topic, client_id):
        depth_topic = rospy.names.resolve_name_without_node_name(depth_topic)
        rgb_topic = rospy.names.resolve_name_without_node_name(rgb_topic)

        if depth_topic in self._converter_clients:
            try:
                self._converter_clients[depth_topic].remove(client_id)
            except ValueError:
                pass
            if len(self._converter_clients[depth_topic]) == 0:
                del self._converter_clients[depth_topic]
                if depth_topic in self._converter_processes:
                    print 'Shutting down converter for {}'.format(depth_topic)
                    self._converter_processes[depth_topic].kill()
                    del self._converter_processes[depth_topic]

        key = (depth_topic, rgb_topic)
        if key in self._encoder_clients:
            try:
                self._encoder_clients[key].remove(client_id)
            except ValueError:
                pass
            if len(self._encoder_clients[key]) == 0:
                del self._encoder_clients[key]
                if key in self._encoder_processes:
                    print 'Shutting down encoder for {}'.format(key)
                    self._encoder_processes[key].kill()
                    del self._encoder_processes[key]

    def shutdown(self):
        for topic, process in self._converter_processes.items():
            print 'Shutting down converter for {}'.format(topic)
            process.kill()
        for key, process in self._encoder_processes.items():
            print 'Shutting down encoder for {}'.format(key)
            process.kill()
        self._converter_clients.clear()
        self._converter_processes.clear()
        self._encoder_clients.clear()
        self._encoder_processes.clear()

