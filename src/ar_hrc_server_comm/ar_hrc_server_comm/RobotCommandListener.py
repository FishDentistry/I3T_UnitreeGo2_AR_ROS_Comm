import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
import yaml 
import os
import requests
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from rclpy.action import ActionClient
from ar_hrc_server_comm.web_request_scripts.FindRequester import get_request_client_with_protocol



class RobotCommandListener(Node):

    def __init__(self):
        super().__init__('robot_command_listener')

        config_package = 'ar_hrc_server_comm'  # <- this is the name of the original package holding the YAML
        package_share_dir = get_package_share_directory(config_package)
        yaml_file = os.path.join(package_share_dir, "config", "config.yaml") # change the yaml file for different robots

        # Load YAML config
        with open(yaml_file, 'r') as f:
            config_data = yaml.safe_load(f)
        self.serverURL = config_data.get('server_url')

        self.declare_parameter('transport_type', 'http')
        transport_type = self.get_parameter('transport_type').value
        self.web_client = get_request_client_with_protocol(transport_type)
        self.commandURL = self.serverURL +"/getoldestrobotdestination?robotNamespace="+self.get_namespace()
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypointClient = ActionClient(self, FollowWaypoints, 'FollowWaypoints')

        self.commandListenerTimer = self.create_timer(0.1, self.requestLatestCommand)
    
    def requestLatestCommand(self):
        try:
            resp = self.web_client.get(self.commandURL)#requests.get(self.commandURL)
            if resp.status_code == 200:
                data = resp.json()
                targetPosition = data["destination"]
                
                if(isinstance(targetPosition[0],list)):
                    waypointsGoalMsg = FollowWaypoints.Goal()
                    for pos in targetPosition:
                        ps = PoseStamped()
                        ps.header.frame_id = 'map'
                        ps.header.stamp = self.get_clock().now().to_msg()
                        ps.pose.position.x = pos[0]
                        ps.pose.position.y = pos[1]
                        ps.pose.position.z = pos[2]
                        ps.pose.orientation.w = 1.0  # facing forward
                        waypointsGoalMsg.poses.append(ps)
                    self.sendWaypointsToNav2(waypointsGoalMsg)
                else:
                    if targetPosition[0] != -1000:
                        x, y, z = targetPosition
                        current_target = (x, y, z)

                        if True:
                            # Build PoseStamped for Nav2
                            pose_stamped = PoseStamped()
                            pose_stamped.header.frame_id = "map"
                            pose_stamped.header.stamp = self.get_clock().now().to_msg()
                            pose_stamped.pose.position.x = x
                            pose_stamped.pose.position.y = y
                            pose_stamped.pose.position.z = z
                            # Facing forward by default
                            pose_stamped.pose.orientation.w = 1.0

                            # Send to Nav2
                            self.send_goal_to_nav2(pose_stamped)

                            

                            self.get_logger().info(
                                f"[Headset -> ROS] New Nav2 goal: ({x}, {y}, {z})"
                            )
            else:
                self.get_logger().debug(
                    f"[Headset -> ROS] GET target position failed, code: {resp.status_code}"
                )
        except Exception as e:
            self.get_logger().error(f"[Headset -> ROS] Exception: {e}")

    def send_goal_to_nav2(self, pose_stamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.nav_to_pose_client.wait_for_server()
        self.nav_to_pose_client.send_goal_async(goal_msg)
    
    def sendWaypointsToNav2(self,waypointsGoalMsg):
        self.waypointClient.wait_for_server()
        self.waypointClient.send_goal_async(waypointsGoalMsg)


def main(args=None):
    rclpy.init(args=args)

    robotCommandListener = RobotCommandListener()

    rclpy.spin(robotCommandListener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robotCommandListener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    

