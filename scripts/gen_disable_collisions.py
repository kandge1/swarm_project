import itertools

links = ['camera_flange','gripper_base','gripper_left1','gripper_left2','gripper_left3',
         'gripper_right1','gripper_right2','gripper_right3','joint6','joint6_flange','g_base']
pairs = itertools.combinations(links, 2)
lines = [f'    <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>' for a, b in pairs]
block = "\n".join(lines) + "\n"

path = "src/mycobot_ros2/mycobot_280/mycobot_280pi_camera_moveit2/config/firefighter.srdf"
with open(path, "r") as f:
    content = f.read()

if "camera_flange" not in content:
    content = content.replace("</robot>", block + "</robot>")
    with open(path, "w") as f:
        f.write(content)
    print("Inserted disable_collisions block.")
else:
    print("Looks like gripper disable_collisions entries already exist — skipping to avoid duplicates.")