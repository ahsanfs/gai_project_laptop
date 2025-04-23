import time
import rclpy
from rclpy.node import Node

from arm1_skills import ARM1_Skills # Gripper

def main(subtasks, coords, begin=False):
    """ Brute Force boolean logic for subtask function sequences subtasks
    Args
    ----------
    subtasks : list, each element is a subtask
    coords : list, each elements is the target coordinate for a subtask

    """
    rclpy.init()
    arm1 = ARM1_Skills()

    if begin == True:
        arm1.arm1_home() # Set gripper to home position

    for t, task in enumerate(subtasks):
        x, y, z = coords[t]
        if task == 'home':
            arm1.gripper()

    arm1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(subtasks=['home'], coords=[[208.19, 370.10, -32.14]], begin=True)