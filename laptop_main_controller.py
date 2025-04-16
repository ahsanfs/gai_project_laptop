"""
Draft script for low-level controller to call functions for task plan

Method :
---------
input lists of sub-tasks (correlate to functions) 
input lists of target coordinates (coordinate with list of sub-tasks, one target for each sub-task, match indicies)

Algorithm : Brute force using boolean logic (We can optimize this later)

Define exact sequence for a given subtask (e.g. insert : move_to(coords), move_to(coords), pick_up(),
                                                        move_to(coords), move_to(coords, insert(target_coords)
                                                         )
Need defined names for possible subtasks for boolean logic (correspond with PDDL actions)


Example :
--------
subtask list : [insert wire, lock screw] 

!!!!!! LIST ORDER SHOULD CORRESPOND TO TASK ORDER !!!!!!

target coordinates : [[0.405, 0.045, -0.002], [0.154, -0.309, -0.064]]
target_coordinates[0] is target coordinate for inserting the wire fork
target_coordinates[1] is the target coordinate for locking the screw



!!!!!!!!!!!! PLEASE DOUBLE CHECK RX, RY, RZ COORDINATES PRIOR TO RUNNING WITH REAL ROBOTS !!!!!!!!!!!!!!
UPDATED 12/6/24
"""
import time
import rclpy
from rclpy.node import Node

from arm1_skills import ARM1_Skills # Gripper
from arm2_skills import ARM2_Skills # Screwdriver

def main(subtasks, coords, begin=False):
    """ Brute Force boolean logic for subtask function sequences subtasks
    Args
    ----------
    subtasks : list, each element is a subtask
    coords : list, each elements is the target coordinate for a subtask

    """
    rclpy.init()
    arm1 = ARM1_Skills()
    # arm2 = ARM2_Skills()

    if begin == True:
        arm1.arm1_home() # Set gripper to home position
    # arm2.arm2_home() # Set screwdriver to home position

    for t, task in enumerate(subtasks):
        x, y, z = coords[t]
        if task == 'pickup':
            tolerance = 0.0001
            # arm1.move_to(x, y , z,  173.33, 0.16, 93.13, velocity=1.0) # Move to grasping position
            # arm1.move_to(x, y, z + 200,  180.0, 0, 90, velocity=1.0)            arm1.move_to(410.49, y, z,  -118.21, -0.61, -90.91, velocity=1.0) # Move to grasping position
            
            
            
            arm1.move_to(208.19, 370.11, 91.36,  -118.21, -0.61, -90.91, velocity=1.7) # Move to grasping position
            
                        
            
            arm1.move_to(208.19, 370.11, -2.45,  -118.21, -0.61, -90.91, velocity=0.8) # Move to grasping position
            
            
            arm1.move_to(x, y, z,  -118.21, -0.61, -90.91, velocity=0.2) # Move to grasping position
            

            while rclpy.ok():
                rclpy.spin_once(arm1)  # Process callback queue
                # Lock the screw 
                if arm1.tool_pose and arm1.is_pose_ready(arm1.tool_pose, [x / 1000, y / 1000, z / 1000], tolerance):
                    arm1.get_logger().info("Tool pose reached target position. Gripping.")
                    if arm1.gripper():  # Run the screwdriver task and wait for it to complete
                        time.sleep(3)
                        arm1.move_to(208.19, 370.11, 21.59,  -118.21, -0.61, -90.91, velocity=1.0) # Move to grasping position
            
                        # arm1.arm1_home()
                        # status = True
                    else:
                    #     # arm2.move_to(x, y, z+0.1, -178.91, -2.69, -87.03, velocity=4.0 )
                    #     arm1.get_logger().info("Gripping task finish. Moving to the next position.")
                    #     # arm2.arm2_home()
                    # else:
                        arm1.get_logger().error("Gripping task failed. Halting further operations.")
                    break
                    # if status == True:
                    #     break
                


            # arm1.pick_up() # Grasp and then lift off of buffer??
            # arm1.grip()

        elif task == 'putdown':
            tolerance = 0.0001
            # arm1.move_to(167.51/1000, 244.41/1000, (28.80/1000) + 0.03, 180.0, 0, -180, velocity=1.5) # Move to putting position
            # arm1.move_to(167.51/1000, 244.41/1000, (28.80/1000) + 0.01, 180.0, 0, -180, velocity=1.0) # Move to putting position
            # arm1.move_to(x, y, z, 180.0, 0, -180, velocity=0.5) # Move to putting position


            arm1.move_to(410.49, 405.22, 21.59,  -118.21, -0.61, -90.91, velocity=1.2) # Move to grasping position
            
            arm1.move_to(x, y, z,  -118.21, -0.61, -90.91, velocity=0.3) # Move to grasping position
            
            # arm1.put_down()
            # arm1.release()
            while rclpy.ok():
                rclpy.spin_once(arm1)  # Process callback queue
                # Lock the screw 
                if arm1.tool_pose and arm1.is_pose_ready(arm1.tool_pose, [x / 1000, y / 1000, z / 1000], tolerance):
                    arm1.get_logger().info("Tool pose reached target position. Gripping.")
                    if arm1.gripper():  # Run the screwdriver task and wait for it to complete
                        time.sleep(3)
                        # arm1.arm1_home()
                        # status = True
                    else:
                    #     # arm2.move_to(x, y, z+0.1, -178.91, -2.69, -87.03, velocity=4.0 )
                    #     arm1.get_logger().info("Gripping task finish. Moving to the next position.")
                    #     # arm2.arm2_home()
                    # else:
                        arm1.get_logger().error("Gripping task failed. Halting further operations.")
                    break
        
        # elif taslk == 'gripper':
        #     arm1.gripper()

        elif task == 'insert':
            arm1.move_to(x, y-0.1, z, 173.33, 0.16, 93.13, velocity=1.0)
            arm1.insert(x, y, z, 166.37, 2.67, 177.51, velocity=0.5)
            arm1.arm1_home()
        
        elif task == 'home':
            # arm1.move_to(x, y-0.1, z, 173.33, 0.16, 93.13, velocity=1.0)
            # arm1.insert(x, y, z, 166.37, 2.67, 177.51, velocity=0.5)
            # arm1.arm1_home()
            print("you are there")
            arm1.gripper()

        elif task == 'lock':
            tolerance = 0.0001
            print(task)
            print(x, y, z)
            arm2.move_to(x, y, z+0.284, -178.91, -2.69, 0.94, velocity=4.0) # Initial move to approach (x,y) position
            print('First position')
            arm2.move_to(x, y, z+0.284, -178.91, -2.69, -87.03, velocity=4.0) # Change tool from camera to screwdriver
            print('Second position')
            arm2.move_to(x, y, z+0.1, -178.91, -2.69, -87.03, velocity=4.0 ) # Approach z position (screwhead location)
            print('Third position')
            arm2.move_to(x, y, z, -178.91, -2.69, -87.03, velocity=1.0) # Reach target position
            print('Fourth position')
            arm2.get_logger().info("Waiting for tool_pose to reach target position...")
            while rclpy.ok():
                rclpy.spin_once(arm2)  # Process callback queue
                # Lock the screw 
                if arm2.tool_pose and arm2.is_pose_ready(arm2.tool_pose, [x, y, z], tolerance):
                    arm2.get_logger().info("Tool pose reached target position. Executing lock_screw.")
                    if arm2.call_lock_screw():  # Run the screwdriver task and wait for it to complete
                        time.sleep(3)
                        arm2.move_to(x, y, z+0.1, -178.91, -2.69, -87.03, velocity=4.0 )
                        arm2.get_logger().info("Screwdriver task done. Moving to the next position.")
                        arm2.arm2_home()
                    else:
                        arm2.get_logger().error("Screwdriver task failed. Halting further operations.")
                    break

    arm1.destroy_node()
    # arm2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # main(subtasks=['pickup'], coords=[[200.31, 420.08, 183.10]]) # Screw on terminal 5 coordinates
    main(subtasks=['home'], coords=[[208.19, 370.10, -32.14]], begin=True) # Screw on terminal 5 coordinates
    # main(subtasks=['putdown'], coords=[[155.16/1000, 341.54/1000, 9.50/1000]]) # Screw on terminal 5 coordinates
    # main(subtasks=['putdown'], coords=[[444.23, 405.22, 21.59]], begin=False) # Screw on terminal 5 coordinates
    # aplhago, extract data from video, gan ai