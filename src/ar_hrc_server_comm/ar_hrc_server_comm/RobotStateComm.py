import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory
import yaml 
import os
import requests
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped


class RobotStateComm(Node):

    def __init__(self):
        super().__init__('robot_state_comm')

        config_package = 'ar_hrc_server_comm'  # <- this is the name of the original package holding the YAML
        package_share_dir = get_package_share_directory(config_package)
        yaml_file = os.path.join(package_share_dir, "config", "config.yaml") # change the yaml file for different robots

        # Load YAML config
        with open(yaml_file, 'r') as f:
            config_data = yaml.safe_load(f)
        self.serverURL = config_data.get('server_url')
        self.poseURL = self.serverURL +"/sendrobotcurrentpose"
        self.trajectoryURL = self.serverURL + "/sendrobotcurrenttrajectory"

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # Timer to regularly check pose
        self.timer = self.create_timer(0.1, self.sendRobotPose)
#         self.create_subscription(
#     PoseWithCovarianceStamped,
#     '/amcl_pose',
#     self.amcl_pose_callback,
#     10
# )

        
        self.create_subscription(Path,"/plan",self.sendRobotTrajectory,qos_profile=QoSProfile(depth=10))

    def sendRobotPose(self):
        try:
            now = rclpy.time.Time()
            if self.tfBuffer.can_transform('map', 'base_link', now):
                trans = self.tfBuffer.lookup_transform(
                    target_frame='map',
                    source_frame='base_link',
                    time=now,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                position = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
                rot = trans.transform.rotation
                orientation = [rot.x, rot.y, rot.z, rot.w]
                json = {"namespace":self.get_namespace(),"position":position,"orientation":orientation}
                try:
                    response = requests.post(self.poseURL,json=json)
                except Exception as e:
                    self.get_logger().warn(f"Could not post pose to server: {e}")

        except Exception as e:
            self.get_logger().warn(f"Transform unavailable: {e}")
    
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        try:
            # Extract position from the message
            position = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]

            # Extract orientation (quaternion)
            orientation = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]

            # Build the JSON payload
            json_payload = {
                "namespace": self.get_namespace(),
                "position": position,
                "orientation": orientation
            }

            # Send to external server
            try:
                response = requests.post(self.poseURL, json=json_payload)
            except Exception as e:
                self.get_logger().warn(f"Could not post pose to server: {e}")

        except Exception as e:
            self.get_logger().warn(f"Error processing AMCL pose: {e}")
     

    def sendRobotTrajectory(self, msg: Path):
        trajectory = []
        for poseStamped in msg.poses:
            position = [poseStamped.pose.position.x, poseStamped.pose.position.y,poseStamped.pose.position.z]
            trajectory.append(position)

        payload = {"namespace":self.get_namespace(),"trajectory": trajectory}

        try:
            requests.post(self.trajectoryURL, json=payload)
        except Exception as e:
            self.get_logger().warn(f"Could not post trajectory to server: {e}")


def main(args=None):
    rclpy.init(args=args)

    robotStateComm = RobotStateComm()

    rclpy.spin(robotStateComm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robotStateComm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()