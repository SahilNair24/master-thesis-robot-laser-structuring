import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class CollisionPublisher(Node):
    def __init__(self):
        super().__init__("collision_object_publisher")

        # Create a publisher to the planning_scene topic
        self.planning_scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.timer = self.create_timer(2.0, self.publish_collision_object)

    def publish_collision_object(self):
        self.get_logger().info("Publishing collision object...")

        # Create the collision object
        collision_object = CollisionObject()
        collision_object.id = "box1"
        collision_object.header = Header()
        collision_object.header.frame_id = "base_link"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.5, 0.5, 0.05]  # x, y, z

        pose = Pose()
        pose.position.x = 1.5
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        # Wrap it in a PlanningScene message
        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(collision_object)
        planning_scene_msg.is_diff = True  # important!

        # Publish
        self.planning_scene_pub.publish(planning_scene_msg)
        self.get_logger().info("Collision object published.")
        self.timer.cancel()


def main():
    rclpy.init()
    node = CollisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
