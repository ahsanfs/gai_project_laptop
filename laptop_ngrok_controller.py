from flask import Flask, request, jsonify
import rclpy
import threading
import time
from arm1_skills import ARM1_Skills

app = Flask(__name__)

COORDINATES = {
    "blue_wire": [208.19, 370.10, -32.14],
    "power_supply_5": [444.23, 405.22, 21.59],
}

task_status = {"status": "idle", "current_task": None}  # To track the current task and status


def execute_subtasks(subtasks, positions):
    """
    Executes the given subtasks at specified positions using predefined coordinates.
    Args:
        subtasks (list): List of actions to perform.
        positions (list): List of positions corresponding to the subtasks.
    """
    global task_status
    rclpy.init()
    arm1 = ARM1_Skills()

    try:
        task_status["status"] = "running"
        for subtask, position in zip(subtasks, positions):
            coords = COORDINATES.get(position, [0, 0, 0])  # Fallback to default coords if position not found
            task_status["current_task"] = f"{subtask} at {position}"
            print(f"Executing {subtask} at {position} with coordinates {coords}")

            if subtask == "pickup":
                print("Run pickup", coords)
                main(subtasks=["pickup"], coords=[[208.19, 370.10, -32.14]], begin=True)
            elif subtask == "insert":
                print("Run insert", coords)
                main(subtasks=["putdown"], coords=[[444.23, 405.22, 21.59]], begin=False)
            elif subtask == "home":
                print("Run home", coords)
                main(subtasks=["home"], coords=[[444.23, 405.22, 21.59]], begin=False)
            else:
                print(f"Unsupported subtask: {subtask}")

        task_status["status"] = "completed"
        task_status["current_task"] = None
    except Exception as e:
        print(f"Error executing subtasks: {e}")
        task_status["status"] = "error"
    finally:
        arm1.destroy_node()
        rclpy.shutdown()


def main(subtasks, coords, begin=False):
    """
    Executes the low-level control for subtasks.
    Args:
        subtasks (list): List of subtasks.
        coords (list): List of coordinates for the subtasks.
        begin (bool): Whether to initialize the robot to home position.
    """
    arm1 = ARM1_Skills()

    if begin:
        arm1.arm1_home()

    for t, task in enumerate(subtasks):
        x, y, z = coords[t]
        if task == "pickup":
            tolerance = 0.0001
            arm1.move_to(208.19, 370.11, 91.36, -118.21, -0.61, -90.91, velocity=1.7)
            arm1.move_to(208.19, 370.11, -2.45, -118.21, -0.61, -90.91, velocity=0.8)
            arm1.move_to(x, y, z, -118.21, -0.61, -90.91, velocity=0.2)

            while rclpy.ok():
                rclpy.spin_once(arm1)
                if arm1.tool_pose and arm1.is_pose_ready(arm1.tool_pose, [x / 1000, y / 1000, z / 1000], tolerance):
                    arm1.get_logger().info("Tool pose reached target position. Gripping.")
                    if arm1.gripper():
                        time.sleep(3)
                        arm1.move_to(208.19, 370.11, 21.59, -118.21, -0.61, -90.91, velocity=1.0)
                    else:
                        arm1.get_logger().error("Gripping task failed. Halting further operations.")
                    break

        elif task == "putdown":
            tolerance = 0.0001
            arm1.move_to(410.49, 405.22, 21.59, -118.21, -0.61, -90.91, velocity=1.2)
            arm1.move_to(x, y, z, -118.21, -0.61, -90.91, velocity=0.3)

            while rclpy.ok():
                rclpy.spin_once(arm1)
                if arm1.tool_pose and arm1.is_pose_ready(arm1.tool_pose, [x / 1000, y / 1000, z / 1000], tolerance):
                    arm1.get_logger().info("Tool pose reached target position. Gripping.")
                    if arm1.gripper():
                        time.sleep(3)
                    else:
                        arm1.get_logger().error("Gripping task failed. Halting further operations.")
                    break

        elif task == "home":
            arm1.arm1_home()


@app.route("/arm1_command", methods=["POST"])
def receive_command():
    """
    Receives commands via POST request and queues robot actions.
    """
    try:
        data = request.get_json()
        print("Received data:", data)

        if not data or "action" not in data or "position" not in data:
            return jsonify({"error": "Invalid data format"}), 400

        actions = data["action"]
        positions = data["position"]

        if len(actions) != len(positions):
            return jsonify({"error": "Mismatch between actions and positions"}), 400

        # Run the task in a separate thread
        thread = threading.Thread(target=execute_subtasks, args=(actions, positions))
        thread.start()

        return jsonify({"status": "Task submitted"}), 200
    except Exception as e:
        print(f"Error: {e}")
        return jsonify({"error": "An error occurred", "details": str(e)}), 500


@app.route("/arm1_status", methods=["GET"])
def get_status():
    print("Job finished")
    return jsonify(task_status), 200

@app.route("/arm1_command_back", methods=["POST"])
def receive_command_back():
    try:
        data = request.get_json()
        print("Received data:", data)

        if not data or "action" not in data or "position" not in data:
            return jsonify({"error": "Invalid data format"}), 400

        actions = data["action"]
        positions = data["position"]

        if len(actions) != len(positions):
            return jsonify({"error": "Mismatch between actions and positions"}), 400

        # Run the task in a separate thread
        thread = threading.Thread(target=execute_subtasks, args=(actions, positions))
        thread.start()

        return jsonify({"status": "Task submitted"}), 200
    except Exception as e:
        print(f"Error: {e}")
        return jsonify({"error": "An error occurred", "details": str(e)}), 500

if __name__ == "__main__":
    app.run(debug=True, host="127.0.0.3", port=9000)
