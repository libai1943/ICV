#!/usr/bin/env python3
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, PolygonStamped, Point32
from functools import partial

from visualization import Visualization, ScatterType
from vehicle_param import VehicleParams
from sandbox_msgs.msg import AprilObject
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from jsk_recognition_msgs.msg import PolygonArray


class MarkerSet:
    def __init__(self):
        self.markers = {}
        self.ready = False

    def get_available_points(self):
        points = [
            [msg.pose.position.x, msg.pose.position.y,
             msg.pose.position.z]
            for marker_id, msg in self.markers.items()
            if marker_id > 0 and msg is not None
        ]
        return np.asarray(points) if points else np.empty((0, 3))

    def check_ready(self):
        self.ready = bool(self.markers) and all(
            msg is not None for msg in self.markers.values())

rad_offset = 0 # np.pi / 2

def rotate_z(rot: Rotation, rad):
    [r, p, y] = rot.as_euler('xyz')
    return rot.from_euler('xyz', [r, p, y + rad])

class Vehicle(MarkerSet):
    def __init__(self):
        super().__init__()
        self.rear_center = None
        self.twist = None
        self.config = VehicleParams

        z_min = -self.config['marker_height']
        z_max = self.config['height'] - self.config['marker_height']

        self.footprint = [
            [-self.config['rear_hang'], -self.config['width'] / 2, z_min],
            [self.config['wheel_base'] + self.config['front_hang'], -self.config['width'] / 2, z_min],
            [self.config['wheel_base'] + self.config['front_hang'], self.config['width'] / 2, z_min],
            [-self.config['rear_hang'], self.config['width'] / 2, z_min],
        ]

    def get_pose(self):
        q = self.markers[0].pose.orientation
        _, _, yaw = Rotation.from_quat(
            [q.x, q.y, q.z, q.w]).as_euler("xyz")
        p = self.markers[self.rear_center].pose.position
        return p.x, p.y, yaw + rad_offset

    def get_footprint(self):
        set_ori = self.markers[0].pose.orientation
        r = rotate_z(Rotation.from_quat([set_ori.x, set_ori.y, set_ori.z, set_ori.w]), rad_offset)
        points = r.apply(self.footprint)
        marker_pose = self.markers[self.rear_center].pose
        points[:, 0] += marker_pose.position.x
        points[:, 1] += marker_pose.position.y
        points[:, 2] += marker_pose.position.z
        return points

    def find_rear_center(self):
        body = self.markers.get(0)
        if body is None:
            return False
        q = body.pose.orientation
        _, _, yaw = Rotation.from_quat(
            [q.x, q.y, q.z, q.w]).as_euler("xyz")
        yaw += rad_offset
        heading = np.array([np.cos(yaw), np.sin(yaw)])
        candidates = [
            (key, np.array([msg.pose.position.x,
                            msg.pose.position.y]))
            for key, msg in self.markers.items()
            if key > 0 and msg is not None
        ]
        if not candidates:
            return False
        self.rear_center = min(
            candidates, key=lambda item: item[1] @ heading)[0]
        return True

    def check_ready(self):
        self.ready = bool(self.markers) and all(
            msg is not None for msg in self.markers.values())
        if self.ready and self.rear_center is None:
            self.ready = self.find_rear_center()

class Polygon(MarkerSet):
    def __init__(self):
        super().__init__()

    def convex_hull(self):
        points = self.get_available_points()
        if points.shape[0] > 2:
            hull = ConvexHull(points[:, :2])
            return points[hull.vertices, :2]
        else:
            return np.empty(shape=(0, 2))

class Localization:
    def __init__(self, ns):
        self.boundary = Polygon()
        self.vehicles = {}
        self.obstacles = {}
        self.topics = {}
        self.visual = Visualization('nokov_markers', 'world')
        self.info_pub = rospy.Publisher('/nokov_info', String, queue_size=10, latch=True)
        self.object_pub = rospy.Publisher('/object', AprilObject, queue_size=10, latch=True)
        self.polygon_pub = rospy.Publisher('/obstacles', PolygonArray, queue_size=10, latch=True)

        topics = rospy.get_published_topics(ns)
        for topic, _ in topics:
            _, name, sub_topic = topic[1:].split('/')
            marker_info = name.split('_')
            marker_idx = 0 # identifies marker set, real markers starts with 1
            if len(marker_info) < 2:
                print('MarkerSet %s detected' % name)
                marker_set = name
            else:
                marker_set, marker = marker_info
                try:
                    marker_idx = int(marker.replace('Marker', ''))
                except:
                    print(marker)

            topic_type = PoseStamped if sub_topic == 'pose' else TwistStamped
            topic_callback = None
            if marker_set.startswith('Boundary'):
                self.boundary.markers[marker_idx] = None
                if sub_topic == 'pose':
                    topic_callback = partial(self.boundary_callback, marker_idx)

            elif marker_set.startswith('Vehicle'):
                vehicle_idx = int(marker_set.replace('Vehicle', ''))
                self.vehicles.setdefault(vehicle_idx, Vehicle()).markers[marker_idx] = None

                if sub_topic == 'pose':
                    topic_callback = partial(
                        self.vehicle_callback, vehicle_idx, marker_idx)
                elif sub_topic == 'twist' and marker_idx == 0:
                    topic_callback = partial(self.vehicle_twist_callback, vehicle_idx)
            elif marker_set.startswith('Obstacle'):
                obstacle_idx = int(marker_set.replace('Obstacle', ''))
                self.obstacles.setdefault(obstacle_idx, Polygon()).markers[marker_idx] = None
                if sub_topic == 'pose':
                    topic_callback = partial(self.obstacle_callback, obstacle_idx, marker_idx)
            else:
                print('unknown marker set %s' % marker_set)
                continue

            if not topic_callback:
                continue

            self.topics[topic] = rospy.Subscriber(topic, topic_type, topic_callback, queue_size=1)

    def boundary_callback(self, marker_idx, msg):
        self.boundary.markers[marker_idx] = msg
        self.boundary.check_ready()

    def obstacle_callback(self, obstacle_idx, marker_idx, msg):
        self.obstacles[obstacle_idx].markers[marker_idx] = msg
        self.obstacles[obstacle_idx].check_ready()

    def vehicle_callback(self, vehicle_idx, marker_idx, msg):
        self.vehicles[vehicle_idx].markers[marker_idx] = msg
        self.vehicles[vehicle_idx].check_ready()

    def vehicle_twist_callback(self, vehicle_idx, msg):
        self.vehicles[vehicle_idx].twist = msg

    def spin_once(self):
        loc_info = {
            'boundary': [],
            'obstacles': [],
            'vehicles': {},
        }

        points = self.boundary.get_available_points()
        self.visual.scatter(points[:, 0], points[:, 1], size=0.02, type=ScatterType.Sphere, ns='boundary points')

        if self.boundary.ready:
            boundary_min = np.min(points, axis=0)
            boundary_max = np.max(points, axis=0)
            boundary = np.array([
                [boundary_min[0], boundary_min[1]],
                [boundary_min[0], boundary_max[1]],
                [boundary_max[0], boundary_max[1]],
                [boundary_max[0], boundary_min[1]],
            ])
            loc_info['boundary'] = [boundary_min[0], boundary_max[0], boundary_min[1], boundary_max[1]]

            self.visual.plot(boundary[:, 0], boundary[:, 1], width=0.01, color=(1.0, 1.0, 1.0, 0.2), ns='boundary', close=True)

        points = np.zeros((0, 3))
        polygons = PolygonArray()
        polygons.header.stamp = rospy.Time.now()
        polygons.header.frame_id = 'world'
        for k, ob in self.obstacles.items():
            points = np.vstack((points, ob.get_available_points()))
            if ob.ready:
                vertices = ob.convex_hull()
                loc_info['obstacles'].append(vertices.tolist())

                self.visual.plot(vertices[:, 0], vertices[:, 1], width=0.01, color=(0.9, 0, 0.7), ns='obstacle', id=k, close=True)

                poly = PolygonStamped()
                poly.header = polygons.header
                poly.polygon.points = [Point32(x=v[0], y=v[1], z=0.0) for v in vertices]
                polygons.polygons.append(poly)

        if len(polygons.polygons) > 0:
            self.polygon_pub.publish(polygons)

        self.visual.scatter(points[:, 0], points[:, 1], size=0.02, type=ScatterType.Sphere, ns='obstacle points')

        points = np.zeros((0, 3))
        rear_points = np.empty((0, 3))
        for i, (k, veh) in enumerate(self.vehicles.items()):
            if veh.ready and veh.rear_center is not None:
                vertices = veh.get_footprint()
                self.visual.plot(vertices[:, 0], vertices[:, 1], width=0.01, ns='vehicle', color=(0.5, 0.2, 1), id=k, close=True)

                pose = veh.get_pose()
                loc_info['vehicles'][k] = pose
                april_msg = AprilObject(type=AprilObject.VEHICLE, id=k, x=pose[0], y=pose[1], yaw=pose[2])
                april_msg.header.stamp = rospy.Time.now()
                april_msg.header.frame_id = "world"
                self.object_pub.publish(april_msg)

                self.visual.arrow(pose[0], pose[1], pose[2], color=(0.5, 0.9, 0.2), length=0.05)

            for km, v in veh.markers.items():
                if km > 0 and v:
                    pos = v.pose.position
                    if km == veh.rear_center:
                        rear_points = np.vstack((rear_points, [pos.x, pos.y, pos.z]))
                    else:
                        points = np.vstack((points, [pos.x, pos.y, pos.z]))

        self.visual.scatter(points[:, 0], points[:, 1], size=0.02, type=ScatterType.Sphere, ns='vehicle points')
        self.visual.scatter(rear_points[:, 0], rear_points[:, 1], size=0.02, ns='vehicle rear points', type=ScatterType.Sphere)

        self.info_pub.publish(String(data=json.dumps(loc_info)))


if __name__ == '__main__':
    rospy.init_node('nokov_localization')
    namespace = "/vrpn_client_node"
    rospy.sleep(2.0)
    loc = Localization(namespace)
    timer = rospy.Timer(rospy.Duration(1.0 / 60.0),
                        lambda event: loc.spin_once())
    rospy.spin()
