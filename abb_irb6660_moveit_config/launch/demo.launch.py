from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("abb_irb6660", package_name="abb_irb6660_moveit_config").to_moveit_configs()
    # moveit_config = MoveItConfigsBuilder(robot_name="abb_irb6660", package_name="abb_irb6660_moveit_config").moveit_cpp(file_path=get_package_share_directory("moveit2_tutorials")+"/config/motion_planning_python_api_tutorial.yaml").to_moveit_configs()
    return generate_demo_launch(moveit_config)
