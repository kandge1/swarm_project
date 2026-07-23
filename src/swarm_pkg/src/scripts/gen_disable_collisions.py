import itertools
import os
from ament_index_python.packages import get_package_share_directory

links = ['camera_flange','gripper_base','gripper_left1','gripper_left2','gripper_left3',
         'gripper_right1','gripper_right2','gripper_right3','joint6','joint6_flange','g_base']
pairs = itertools.combinations(links, 2)
lines = [f'    <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>' for a, b in pairs]
block = "\n".join(lines) + "\n"

# Find the package and get the path to firefighter.srdf
pkg_share = get_package_share_directory('mycobot_280pi_camera_moveit2')
path = os.path.join(pkg_share, 'config', 'firefighter.srdf')

with open(path, "r") as f:
    content = f.read()

if "camera_flange" not in content:
    content = content.replace("</robot>", block + "</robot>")
    with open(path, "w") as f:
        f.write(content)
    print(f"Inserted disable_collisions block at {path}")
else:
    print("Looks like gripper disable_collisions entries already exist — skipping to avoid duplicates.")