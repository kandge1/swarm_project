from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("firefighter")
        .robot_description(file_path="config/firefighter.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    return generate_spawn_controllers_launch(moveit_config)
