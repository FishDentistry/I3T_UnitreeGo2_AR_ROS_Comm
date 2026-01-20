#!/usr/bin/env python3
# robot_objects_viz_node.py
import os
import math
import requests
from typing import Dict, Set, Tuple

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray

class RobotObjectsViz(Node):
    def __init__(self):
        super().__init__('robot_objects_viz')

        # --- Parameters (declare with defaults) ---
        self.declare_parameter('endpoint_url', 'http://192.168.1.227:8000/getrobotobjectsrobot')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('refresh_hz', 2.0)
        self.declare_parameter('marker_scale', 0.6)  # was 0.25
        self.declare_parameter('text_scale', 0.5)    # was 0.25
        self.declare_parameter('z_offset', 0.2)      # was 0.05
        self.declare_parameter('timeout_sec', 1.5)
        

        self.endpoint_url: str = self.get_parameter('endpoint_url').get_parameter_value().string_value
        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value
        self.refresh_hz: float = self.get_parameter('refresh_hz').get_parameter_value().double_value
        self.marker_scale: float = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.text_scale: float = self.get_parameter('text_scale').get_parameter_value().double_value
        self.timeout_sec: float = self.get_parameter('timeout_sec').get_parameter_value().double_value
        self.z_offset: float = self.get_parameter('z_offset').get_parameter_value().double_value

        # Publisher
        self.pub = self.create_publisher(MarkerArray, 'robot_objects_markers', 10)

        # Track previously published IDs to remove stale markers
        self.prev_ids: Set[int] = set()

        # Timer
        period = 1.0 / max(0.1, self.refresh_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(f'RobotObjectsViz polling {self.endpoint_url} at {self.refresh_hz:.2f} Hz')

    def _tick(self):
        markers = MarkerArray()
        current_ids: Set[int] = set()

        # Fetch data
        try:
            resp = requests.get(self.endpoint_url, timeout=self.timeout_sec)
            resp.raise_for_status()
            data = resp.json()
            if not isinstance(data, list):
                self.get_logger().warn('Endpoint JSON is not a list; skipping this cycle.')
                self._publish_deletes()
                return
        except Exception as e:
            self.get_logger().warn(f'HTTP error: {e}')
            # On failure, clear all existing markers so RViz doesn’t show stale info
            self._publish_deletes()
            return

        # Filter out the sentinel "None" item if present
        objects = []
        for obj in data:
            try:
                cls = obj.get('class', '')
                pos = obj.get('robotMapPosition', None)
                obj_id = obj.get('objID', None)
                if cls == 'None' or pos is None or obj_id is None:
                    continue
                if not (isinstance(pos, list) and len(pos) >= 3):
                    continue
                # Valid object
                objects.append(obj)
            except Exception:
                continue

        # Create markers
        for obj in objects:
            obj_id = int(obj['objID'])
            current_ids.add(obj_id)

            x, y, z = obj['robotMapPosition'][:3]
            # Some pipelines might use mm or cm; assume meters here (as typical in map frame)

            # Choose color by hazard status
            is_hazard = str(obj.get('isHazard', 'False')).lower() == 'true'
            if is_hazard:
                color = (1.0, 0.2, 0.2, 0.95)  # red-ish
            else:
                color = (0.2, 0.8, 0.2, 0.95)  # green-ish

            # 1) Sphere marker
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'robot_objects'
            sphere.id = obj_id * 2          # unique ids (sphere/text pairs)
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = float(z)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self.marker_scale
            sphere.scale.y = self.marker_scale
            sphere.scale.z = self.marker_scale
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            sphere.lifetime = Duration(sec=1)  # auto-expire if not refreshed
            markers.markers.append(sphere)

            # 2) Text marker (class name & id)
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = sphere.header.stamp
            text.ns = 'robot_objects_labels'
            text.id = obj_id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = float(z) + self.marker_scale * 0.6 + self.z_offset
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_scale
            text.color.r, text.color.g, text.color.b, text.color.a = (1.0, 1.0, 1.0, 0.95)
            label_cls = obj.get('class', 'object')
            text.text = f'{label_cls} (#{obj_id})'
            text.lifetime = Duration(sec=1)
            markers.markers.append(text)

        # Publish deletes for any markers not present this cycle
        stale_ids = self.prev_ids - current_ids
        for stale_id in stale_ids:
            # delete sphere
            del_s = Marker()
            del_s.header.frame_id = self.frame_id
            del_s.header.stamp = self.get_clock().now().to_msg()
            del_s.ns = 'robot_objects'
            del_s.id = stale_id * 2
            del_s.action = Marker.DELETE
            markers.markers.append(del_s)

            # delete text
            del_t = Marker()
            del_t.header.frame_id = self.frame_id
            del_t.header.stamp = del_s.header.stamp
            del_t.ns = 'robot_objects_labels'
            del_t.id = stale_id * 2 + 1
            del_t.action = Marker.DELETE
            markers.markers.append(del_t)

        # Publish all updates
        self.pub.publish(markers)
        self.prev_ids = current_ids

    def _publish_deletes(self):
        if not self.prev_ids:
            return
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for stale_id in self.prev_ids:
            del_s = Marker()
            del_s.header.frame_id = self.frame_id
            del_s.header.stamp = now
            del_s.ns = 'robot_objects'
            del_s.id = stale_id * 2
            del_s.action = Marker.DELETE
            markers.markers.append(del_s)

            del_t = Marker()
            del_t.header.frame_id = self.frame_id
            del_t.header.stamp = now
            del_t.ns = 'robot_objects_labels'
            del_t.id = stale_id * 2 + 1
            del_t.action = Marker.DELETE
            markers.markers.append(del_t)

        self.pub.publish(markers)
        self.prev_ids.clear()


def main():
    rclpy.init()
    node = RobotObjectsViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._publish_deletes()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
