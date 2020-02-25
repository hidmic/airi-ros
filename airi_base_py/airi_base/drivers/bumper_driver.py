import rospy

import airi_msgs
import geometry_msgs

import numpy as np
import resource_retriever
import stl

import urdf_parser_py.urdf

from udp_driver import BumperDriver


class BumperDriver(UDPDriver):

    def __init__(self, name='~bumper_driver'):
        super(BumperDriver, self).__init__(name)
        self._bumper_frame = rospy.get_param(rospy.resolve_name('~bumper_frame', name), 'bumper_link')
        self._bumper_contact_publisher = rospy.Publisher(
            'bumper/contacts', airi_msgs.msg.Bumper, queue_size=1
        )
        self._bumper_collision_publisher = rospy.Publisher(
            'bumper/collisions', geometry_msgs.msg.PointCloud, queue_size=1
        )

        robot_description = urdf_parser_py.urdf.Robot.from_parameter_server()
        assert self._bumper_frame in robot_description.link_map, \
            "'{}' frame not found in robot description".format(self._bumper_frame)
        bumper_geometry = robot_description.link_map[self._bumper_frame].collision.geometry
        assert isinstance(bumper_geometry, urdf_parser_py.urdf.Mesh), 'Only mesh collisions are supported'
        mesh_uri = resource_retriever.get_filename(bumper_geometry.filename)
        with resource_retriever.urlopen(mesh_uri) as fh:
            mesh = stl.mesh.Mesh.load(fh)

        mesh_scale = rospy.get_param(rospy.resolve_name('~scale', name), 1.)
        voxel_size = rospy.get_param(rospy.resolve_name('~level_of_detail', name), 0.01)

        centroids = mesh_scale * (mesh.v0 + mesh.v1 + mesh.v2) / 3
        centroids_voxels = np.round(centroids / voxel_size)  # body centered voxel
        voxels = np.unique(centroids_voxels, axis=0)

        collision_msg = geometry_msgs.msg.PointCloud()
        collision_msg.header.frame_id = self._bumper_frame
        for voxel in voxels:
            voxel_centroid = np.mean(centroids[np.where((centroids_voxels == voxel).all(axis=1))], axis=0)
            point_msg = geometry_msgs.msg.Point32()
            point_msg.x = voxel_centroid[0]
            point_msg.y = voxel_centroid[1]
            point_msg.z = voxel_centroid[2]
            collision_msg.points.append(point_msg)
        self._bumper_collision_msg = collision_msg

    def handle_data(self, data):
        now = rospy.get_rostime()
        try:
            left_contact, right_contact = data
        except TypeError, ValueError:
            rospy.logerr('%s got unexpected bumper data: %!s', self.name, data)
            return

        bumper_msg = airi_msgs.msg.Bumper()
        bumper_msg.left_contact = left_contact
        bumper_msg.right_contact = right_contact
        self._bumper_contact_publisher.publish(bumper_msg)

        if left_contact or right_contact:
            self._bumper_collision_msg.stamp = now
            self._bumper_collision_publisher.publish(self._bumper_collision_msg)

