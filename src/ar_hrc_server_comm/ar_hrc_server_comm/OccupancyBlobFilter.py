#!/usr/bin/env python3
# occupancy_blob_filter.py
# Filters small occupied blobs out of a slam_toolbox OccupancyGrid and republishes it.

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from scipy import ndimage as ndi  # pip install scipy

class OccupancyBlobFilter(Node):
    def __init__(self):
        super().__init__("occupancy_blob_filter")

        # ------- Parameters -------
        self.declare_parameter("map_in", "/map")
        self.declare_parameter("map_out", "/map_filtered")
        self.declare_parameter("occ_thresh", 65)            # cells >= occ_thresh are 'occupied'
        self.declare_parameter("open_kernel_size", 0)       # 0 disables; else 3,5,... for binary opening
        self.declare_parameter("min_cluster_cells", 0)      # drop blobs smaller than this many cells
        self.declare_parameter("min_cluster_area_m2", 0.0)  # OR use physical area; 0 disables
        self.declare_parameter("keep_largest_only", False)  # keep only the largest occupied component
        self.declare_parameter("set_removed_to_unknown", False)  # otherwise set removed cells to free (0)

        self.map_in  = self.get_parameter("map_in").value
        self.map_out = self.get_parameter("map_out").value
        self.occ_thresh = int(self.get_parameter("occ_thresh").value)
        self.open_kernel_size = int(self.get_parameter("open_kernel_size").value)
        self.min_cluster_cells = int(self.get_parameter("min_cluster_cells").value)
        self.min_cluster_area_m2 = float(self.get_parameter("min_cluster_area_m2").value)
        self.keep_largest_only = bool(self.get_parameter("keep_largest_only").value)
        self.set_removed_to_unknown = bool(self.get_parameter("set_removed_to_unknown").value)

        # Latching-like QoS for maps (so RViz gets the last one)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(OccupancyGrid, self.map_out, qos)
        self.sub = self.create_subscription(OccupancyGrid, self.map_in, self.on_map, 1)

        self.get_logger().info(
            f"OccupancyBlobFilter listening on {self.map_in} → publishing {self.map_out}"
        )

    def on_map(self, msg: OccupancyGrid):
        res = float(msg.info.resolution)
        w, h = msg.info.width, msg.info.height

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)
        unknown_mask = (data < 0)
        occ_mask = (data >= self.occ_thresh)

        # Optional morphological opening to knock out single pixels/thin spikes
        if self.open_kernel_size and self.open_kernel_size >= 3:
            k = self.open_kernel_size
            se = np.ones((k, k), dtype=bool)
            occ_mask = ndi.binary_opening(occ_mask, structure=se)

        # Connected components (8-connectivity)
        labeled, n = ndi.label(occ_mask, structure=np.ones((3, 3), dtype=bool))
        if n == 0:
            # Nothing occupied; publish original with any opening applied (clears speckle)
            new_data = data.copy()
            new_data[np.logical_and(~occ_mask, data >= self.occ_thresh)] = 0 if not self.set_removed_to_unknown else -1
            self._publish(msg, new_data)
            return

        # Compute per-component sizes in cells
        sizes = ndi.sum(occ_mask, labeled, index=np.arange(1, n + 1))
        sizes = np.asarray(sizes, dtype=np.int64)  # length n, components are 1..n

        # Determine minimum size in cells
        min_cells = int(self.min_cluster_cells) if self.min_cluster_cells > 0 else 0
        if self.min_cluster_area_m2 > 0.0:
            area_cells = int(math.ceil(self.min_cluster_area_m2 / (res * res)))
            min_cells = max(min_cells, area_cells)

        # Build keep set
        keep_labels = set()
        if self.keep_largest_only:
            # pick the largest component
            keep_labels.add(int(np.argmax(sizes) + 1))
        else:
            # keep components >= min_cells; if min_cells==0 keep all
            if min_cells == 0:
                keep_labels = set(range(1, n + 1))
            else:
                large = np.where(sizes >= min_cells)[0] + 1
                keep_labels = set(large.tolist())

        # Construct new data: start from original, then zero/unknown out small blobs
        new_data = data.copy()
        # mask of occupied cells we want to remove
        remove_mask = np.logical_and(occ_mask, ~np.isin(labeled, list(keep_labels)))
        new_data[remove_mask] = -1 if self.set_removed_to_unknown else 0

        # Preserve unknowns where they were unknown originally
        new_data[unknown_mask] = -1

        # Publish
        self._publish(msg, new_data)

    def _publish(self, src_msg: OccupancyGrid, grid: np.ndarray):
        out = OccupancyGrid()
        out.header = src_msg.header  # keep same frame/time
        out.info   = src_msg.info
        out.data   = grid.astype(np.int16).ravel().tolist()
        self.pub.publish(out)

def main():
    rclpy.init()
    node = OccupancyBlobFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
