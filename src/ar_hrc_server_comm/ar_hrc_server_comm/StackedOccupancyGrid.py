
#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import requests
from ament_index_python import get_package_share_directory
import yaml
import os 

import open3d as o3d

class StackedOccupancyGrid(Node):
    def __init__(self):
        super().__init__('slam2d_to_voxel3d_extruder')

        # --- Parameters (existing) ---
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 1.5)
        self.declare_parameter('dz', 0.1)                      # vertical voxel size
        self.declare_parameter('occ_thresh', 65)               # >= occupied
        self.declare_parameter('free_thresh', 25)              # <= free (unused by default)
        self.declare_parameter('publish_every_update', True)   # re-publish whenever map updates

        # --- NEW: mesh-saving controls for cube-per-voxel output ---
        self.declare_parameter('save_mesh_cubes', True)
        self.declare_parameter('mesh_path', 'stacked_voxels_mesh.ply')
        self.declare_parameter('cube_xy_scale_frac', 1.0)      # 0.5..1.0 to make cubes visually slimmer than cell
        self.declare_parameter('cube_z_scale_frac', 1.0)       # 0.5..1.0 to make cubes shorter than dz
        self.declare_parameter('max_voxels_warn', 300000)      # warn if we exceed this many cubes

        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.z_min     = self.get_parameter('z_min').get_parameter_value().double_value
        self.z_max     = self.get_parameter('z_max').get_parameter_value().double_value
        self.dz        = self.get_parameter('dz').get_parameter_value().double_value
        self.occ_thresh= int(self.get_parameter('occ_thresh').get_parameter_value().integer_value)
        self.free_thresh= int(self.get_parameter('free_thresh').get_parameter_value().integer_value)
        self.publish_every_update = self.get_parameter('publish_every_update').get_parameter_value().bool_value

        self.save_mesh_cubes = self.get_parameter('save_mesh_cubes').get_parameter_value().bool_value
        self.mesh_path       = self.get_parameter('mesh_path').get_parameter_value().string_value
        self.cxy_frac        = float(self.get_parameter('cube_xy_scale_frac').get_parameter_value().double_value)
        self.cz_frac         = float(self.get_parameter('cube_z_scale_frac').get_parameter_value().double_value)
        self.max_voxels_warn = int(self.get_parameter('max_voxels_warn').get_parameter_value().integer_value)

        # Latching-like QoS so markers persist in RViz
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'voxel_grid_markers', qos)

        self.sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, 1)
        self.already_published = False

        self.get_logger().info('slam2d_to_voxel3d_extruder ready. Subscribing to %s' % self.map_topic)
        config_package = 'ar_hrc_server_comm'  # <- this is the name of the original package holding the YAML
        package_share_dir = get_package_share_directory(config_package)
        yaml_file = os.path.join(package_share_dir, "config", "config.yaml") # change the yaml file for different robots
        with open(yaml_file, 'r') as f:
            config_data = yaml.safe_load(f)
        self.serverURL = config_data.get('server_url')
        self.voxMapURL = self.serverURL +"/sendvoxelmap"
    
    def sendVoxelMap(self):
        files = {'voxMap': open('stacked_voxels_mesh.ply','rb')}
        values = {'robotNamespace': self.get_namespace()}
        try:
            r = requests.post(self.voxMapURL, files=files, data=values, timeout=2.0)
        except Exception as e:
            self.get_logger().info("Could not post voxel map to server")

    def on_map(self, msg: OccupancyGrid):
        if self.already_published and not self.publish_every_update:
            return

        # Ensure Python floats for ROS msgs
        res = float(msg.info.resolution)
        w, h = msg.info.width, msg.info.height
        ox, oy, oz = float(msg.info.origin.position.x), float(msg.info.origin.position.y), float(msg.info.origin.position.z)

        # Origin yaw (usually 0 with slam_toolbox, but handle it anyway)
        q = msg.info.origin.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        c, s = math.cos(yaw), math.sin(yaw)

        # 2D occupancy mask
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)
        occ_mask = data >= self.occ_thresh
        occ_indices = np.argwhere(occ_mask)  # rows (j), cols (i)

        if occ_indices.size == 0:
            self.get_logger().warn('No occupied cells in 2D map (nothing to extrude).')
            return

        # World coordinates of occupied cell centers (for visualization + cube placement)
        js = occ_indices[:, 0].astype(np.float64)
        is_ = occ_indices[:, 1].astype(np.float64)
        cx = (is_ + 0.5) * res
        cyy = (js  + 0.5) * res
        world_x = ox + (cx * c - cyy * s)
        world_y = oy + (cx * s + cyy * c)

        # Z slice centers
        nz = int(math.ceil((self.z_max - self.z_min) / self.dz))
        if nz <= 0:
            self.get_logger().warn('z-range is empty; adjust z_min/z_max/dz.')
            return
        z_centers = self.z_min + (np.arange(nz, dtype=np.float64) + 0.5) * self.dz

        # ---------------- RViz MarkerArray (original behavior) ----------------
        ma = MarkerArray()
        base_id = 0
        color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.6)  # translucent blue
        dz = float(self.dz)

        for k, zc in enumerate(z_centers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'voxels'
            m.id = base_id + k
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.scale.x = float(res)
            m.scale.y = float(res)
            m.scale.z = float(dz)
            m.color = color
            m.pose.orientation.w = 1.0

            pts = []
            ap = pts.append
            for x_val, y_val in zip(world_x, world_y):
                p = Point()
                p.x = float(x_val)
                p.y = float(y_val)
                p.z = float(zc)
                ap(p)
            m.points = pts
            ma.markers.append(m)

        for m in ma.markers:
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0

        self.marker_pub.publish(ma)
        self.get_logger().info(f'Published voxel grid: {len(occ_indices)} cells extruded into {nz} slices '
                               f'({len(occ_indices)*nz} cubes). Topic: voxel_grid_markers')
        self.already_published = True

        # ---------------- NEW: Build Open3D mesh as cubes-per-voxel ----------------
        if not self.save_mesh_cubes:
            return

        num_xy = world_x.shape[0]
        total_voxels = int(num_xy) * int(nz)
        if total_voxels > self.max_voxels_warn:
            self.get_logger().warn(
                f'About to place {total_voxels} cubes; this may be heavy. '
                f'Consider increasing dz, shrinking z-range, or lowering cube scale.'
            )

        # Cube dimensions (optionally shrunk a bit for visual gap)
        wx = float(self.cxy_frac) * res
        wy = float(self.cxy_frac) * res
        wz = float(self.cz_frac)  * dz

        # Precompute a base cube (origin at 0,0,0). We'll translate to center later.
        base_cube = o3d.geometry.TriangleMesh.create_box(width=wx, height=wy, depth=wz)
        base_verts = np.asarray(base_cube.vertices)  # (8,3)
        base_tris  = np.asarray(base_cube.triangles, dtype=np.int64)  # (12,3)

        # To center the cube at (x,y,z), translate by (-wx/2, -wy/2, -wz/2) + (x,y,z)
        offset_center = np.array([-0.5*wx, -0.5*wy, -0.5*wz], dtype=np.float64)

        verts_list = []
        tris_list  = []
        vcount = 0

        # Build cubes slice-by-slice (keeps peak memory somewhat lower than full tiling)
        for zc in z_centers:
            z_off = float(zc)  # center of this voxel layer
            # For each occupied 2D cell, add one cube centered at (x,y,zc)
            for x_val, y_val in zip(world_x, world_y):
                # translate base verts to this cube's position
                t = np.array([float(x_val), float(y_val), z_off], dtype=np.float64) + offset_center
                verts_list.append(base_verts + t)
                tris_list.append(base_tris + vcount)
                vcount += base_verts.shape[0]

        if len(verts_list) == 0:
            self.get_logger().warn('No cubes to write.')
            return

        V = np.vstack(verts_list).astype(np.float64)
        F = np.vstack(tris_list).astype(np.int32)

        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(V),
            o3d.utility.Vector3iVector(F)
        )
        mesh.compute_vertex_normals()

        # Optional cleanups: merge exact duplicates, drop duplicate faces, etc.
        mesh.remove_duplicated_vertices()
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_unreferenced_vertices()

        ok = o3d.io.write_triangle_mesh(self.mesh_path, mesh, write_triangle_uvs=False, print_progress=False)
        if ok:
            self.get_logger().info(f'Wrote cube-stacked voxel mesh: {self.mesh_path}  '
                                   f'({len(V)} vertices, {len(F)} triangles, {total_voxels} cubes)')
            self.sendVoxelMap()
        else:
            self.get_logger().warn(f'Failed to write mesh to: {self.mesh_path}')
    

def main():
    rclpy.init()
    node = StackedOccupancyGrid()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

