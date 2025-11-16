import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import csv
import time


class FKClient(Node):
    def __init__(self):
        super().__init__('fk_client')
        self.cli = self.create_client(GetPositionFK, '/compute_fk')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FK service...')

    def send_request(self, joint_names, joint_positions, fk_link):
        req = GetPositionFK.Request()
        req.fk_link_names = [fk_link]
        req.robot_state.joint_state.name = joint_names
        req.robot_state.joint_state.position = joint_positions
        req.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = FKClient()

    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    fk_link = 'link_6'  # or 'ee_link', etc.

    input_file = '/home/sahilsnair/Desktop/laser-toolpath/scripts/joint_positions.csv'
    output_file = '/home/sahilsnair/Desktop/laser-toolpath/scripts/fk_results.csv'

    # This just clears the file every time the node is run
    with open(output_file, mode='w', newline='') as file:
        pass

    results = []

    with open(input_file, 'r') as infile:
            reader = csv.reader(infile)
            for row in reader:
                joint_positions = [float(val) for val in row]
                response = node.send_request(joint_names, joint_positions, fk_link)
                if response and response.pose_stamped:
                    pose = response.pose_stamped[0].pose
                    position = pose.position
                    orientation = pose.orientation  
                    x = round(position.x * 1000, 3)
                    y = round(position.y * 1000, 3)
                    z = round(position.z * 1000, 3)
                    ox = round(orientation.x, 3)
                    oy = round(orientation.y, 3)
                    oz = round(orientation.z, 3)
                    results.append([x, y, z, ox, oy, oz])
                else:
                    node.get_logger().warn('Failed to get FK for: {}'.format(joint_positions))
                    results.append([None]*7)  # Fill with Nones if FK failed

    with open(output_file, 'a', newline='') as outfile:
            writer = csv.writer(outfile)
            # writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            writer.writerows(results)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
