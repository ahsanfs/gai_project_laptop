import re
from flask import Flask, request, jsonify
import requests
import threading
import time
from arm1_skills import ARM1_Skills  # Gripper
from arm2_skills import ARM2_Skills  # Screwdriver
import rclpy

app = Flask(__name__)
url_update_graph_and_chat = "http://localhost:6000/update_graph_and_chat"
url_LLM_server = "http://127.0.0.1:7000/LLM_reciever_prompt"

LLM_message="" #"helo I am LLM"
output_pddl_str ="" # "\n(find red_wire)\n(pickup arm1 red_wire table)\n(insert arm1 red_wire power_supply_5)\n(putdown arm1 red_wire power_supply_5)\n(lock arm2 red_wire power_supply_5)"


robots_list=["arm1","arm2"]
actions_list=["pickup","lock", "insert", "putdown","find"]
object_item_list=["red_wire","blue_wire", "yellow_wire", "white_wire","green_wire"]
location_list=["table","power_supply_5"]

extacted_commands=[]


#and this
ros2_initialized = False
arm1 = None
arm2 = None

class PDDL_line:
    def __init__(self, command_keypharse):
        self.action="-"
        self.robot="-"
        self.object_item="-"
        self.location="-"

        for ck in command_keypharse:
            for al in actions_list:
                if ck==al:
                    self.action = ck
                    break
            for rl in robots_list:
                if ck==rl:
                    self.robot = ck
                    break

            for oi in object_item_list:
                if ck==oi:
                    self.object_item = ck
                    break
            for ll in location_list:
                if ck==ll:
                    self.location = ck
                    break

    def __str__(self):
        return f"{self.robot}->{self.action} obj:{self.object_item} on:{self.location})"

def extract_pddl_lines(text):
    lines = re.findall(r'\((.*?)\)', text)
    # print(lines)
    return lines

def extract_basic_keyPharse(text):
    command_keypharse = text.strip('()').split()
    # print(command_keypharse)
    return PDDL_line(command_keypharse)



#This
def execute_subtasks(commands):
    """Executes subtasks sequentially based on the parsed PDDL actions."""
    global arm1, arm2

    for command in commands:
        x, y, z = map(float, [0, 0, 0])  # Default coordinates (can be updated based on object location)
        print(f"Executing {command.action} with {command.robot} at {x}, {y}, {z}")

        if command.action == "pickup" and command.robot == "arm1":
            arm1.move_to(x, y, z, -118.21, -0.61, -90.91, velocity=0.3)
            arm1.gripper()  # Close gripper
            arm1.move_to(x, y, z + 100, -118.21, -0.61, -90.91, velocity=1.0)

        elif command.action == "putdown" and command.robot == "arm1":
            arm1.move_to(x, y, z, -118.21, -0.61, -90.91, velocity=0.3)
            arm1.release()  # Open gripper
            arm1.move_to(x, y, z + 100, -118.21, -0.61, -90.91, velocity=1.0)

        elif command.action == "lock" and command.robot == "arm2":
            arm2.move_to(x, y, z + 100, -178.91, -2.69, -87.03, velocity=1.0)
            arm2.call_lock_screw()  # Lock screw
            arm2.move_to(x, y, z + 200, -178.91, -2.69, -87.03, velocity=1.0)

        else:
            print(f"Action '{command.action}' is not implemented for {command.robot}.")

    print("All tasks executed successfully.")

def convert_PDDL_line_to_jsonGraph(extacted_commands):
    node = []
    link=[]
    robot={'-':{'start':0,'end':0}} #to prevert null value
    last_key_id_on_each_robot={'-':-1}
    has_robot=True


    ### Initialize robot start-end node
    for i in range(len(extacted_commands)):
        has_robot=True
        for rb in robot:
            if extacted_commands[i].robot == rb:
                has_robot=False

        if has_robot:
            start_robot_node=-(2*len(robot)-1)
            end_robot_node=-(2*len(robot))
            # print(start_robot_node,end_robot_node)
            node_entry = {
            "key": start_robot_node,
            "command": "Start_"+str(extacted_commands[i].robot),
            "color": "lightblue",
            "category": "StartEnd"
            }
            node.append(node_entry)
            node_entry = {
            "key": end_robot_node,
            "command": "End_"+str(extacted_commands[i].robot),
            "color": "lightblue",
            "category": "StartEnd"
            }
            node.append(node_entry)
            robot.update({str(extacted_commands[i].robot):{'start':start_robot_node,'end':end_robot_node}})
            last_key_id_on_each_robot.update({str(extacted_commands[i].robot):-1})
    # print(robot)
    give_key_id=0
    prev_robot_active="-"
    # print(last_key_id_on_each_robot)
   
    for i in range(len(extacted_commands)):
        # Draw node
        if (i==0):
            #print(list(robot.keys())[1])
            prev_robot_active=list(robot.keys())[1]
        elif(i>0 and extacted_commands[i].robot!=prev_robot_active):
            ### add wait node for switch robot
            # print(extacted_commands[i].robot)
            node_entry = {
            "key": give_key_id,
            "command": "wait another",
            "color": "white",
            "category": "Process"
            }
            node.append(node_entry)
            # add link from start robot node to node
            if last_key_id_on_each_robot[extacted_commands[i].robot]<0:
                last_key_id_on_each_robot[extacted_commands[i].robot]=robot[extacted_commands[i].robot]["start"]

                link_entry = {
                    "key": str(-give_key_id-1)+"s", 
                    "from": last_key_id_on_each_robot[extacted_commands[i].robot], 
                    "to": give_key_id
                }
                
                last_key_id_on_each_robot[extacted_commands[i].robot]=give_key_id
                link.append(link_entry)

            # add link from previous process node to node
            link_entry = {
                    "key": -give_key_id-1, 
                    "from": last_key_id_on_each_robot[extacted_commands[i-1].robot], 
                    "to": give_key_id
                }
                
            last_key_id_on_each_robot[extacted_commands[i].robot]=give_key_id
            link.append(link_entry)
            give_key_id=give_key_id+1

            prev_robot_active=extacted_commands[i].robot
            #print(prev_robot_active)

        ### add main node
        node_entry = {
            "key": give_key_id,
            "command": extacted_commands[i].action,
            "color": "white",
            "category": "Process"
        }
        node.append(node_entry)
        

        ### Draw link main process
        # draw link on initial
        if (i==0):
            link_entry = {
                "key": str(-give_key_id-1)+"s", 
                "from": robot[list(robot.keys())[1]]["start"], 
                "to": give_key_id
            }
            last_key_id_on_each_robot[extacted_commands[1].robot]=give_key_id
            link.append(link_entry)
        # draw link on each step
        else:
            link_entry = {
                "key": -give_key_id-1, 
                "from": last_key_id_on_each_robot[extacted_commands[i].robot], 
                "to": give_key_id
            }
            last_key_id_on_each_robot[extacted_commands[i].robot]=give_key_id
            link.append(link_entry)

        give_key_id=give_key_id+1
        ### Draw terminator link, node each robot to the end
    for i in range(1,len(last_key_id_on_each_robot)):
        link_entry = {
            "key": str(-give_key_id-1)+"e", 
            "from": last_key_id_on_each_robot[list(last_key_id_on_each_robot.keys())[i]], 
            "to": robot[list(last_key_id_on_each_robot.keys())[i]]["end"]
        }
        link.append(link_entry)

        give_key_id=give_key_id+1


    return node,link

@app.route('/')
def home():
    return jsonify({"status": "Flask app running", "message": "Automatic data sending to Electron enabled"})

# Function to send data automatically
def send_data_to_url(url,data):
    while True:
        try:
            response = requests.post(url, json=data)
            print(f"Data sent: {response.status_code}, Response: {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"Error sending data: {e}")
        time.sleep(10)  # Wait 10 seconds before sending data again

def convert_pddl_and_send_to_electron():
    node = []
    link=[]
    while True:
        pddl_lines=extract_pddl_lines(output_pddl_str)
        for pddl_line in pddl_lines:
            extacted_command=extract_basic_keyPharse(pddl_line)
            extacted_commands.append(extacted_command)
        node,link=convert_PDDL_line_to_jsonGraph(extacted_commands)
        message="okay"
        print(node)
        print("\n")
        print(link)
        post_data = {'message': message,
            'linkDataArray': link,
            'nodeDataArray': node
            }
        print("\njson")
        print(post_data)
        send_data_to_url("p".post_data)

def start_main():
    thread = threading.Thread(target=convert_pddl_and_send_to_electron)
    thread.daemon = True
    thread.start()

@app.route('/user_prompt_to_LLM_server', methods=['POST'])
def user_prompt_to_LLM_server():
    try:
        data = request.get_json()
        if not data or 'message' not in data:
            return jsonify({'error': 'Invalid data format'}), 400

        message = data['message']
        print(f"Received message: {message}")
        post_data = {'message': message}
        print("\njson")
        print(post_data)
        send_data_to_url(url_LLM_server,post_data)

        return jsonify({'status': 'success'}), 200
    except Exception as e:
        print(f"Error: {e}")
        return jsonify({'error': 'An error occurred'}), 500


@app.route('/LLM_message_response', methods=['POST'])
def receive_string():
    try:
        data = request.get_json()
        if not data or 'message' not in data:
            return jsonify({'error': 'Invalid data format'}), 400

        LLM_message = data['message']
        output_pddl_str = data['output_pddl']
        
        print(f"Received message: {LLM_message}")
        print(f"Received output_pddl: {output_pddl_str}")
        pddl_lines=extract_pddl_lines(output_pddl_str)
        for pddl_line in pddl_lines:
            extacted_command=extract_basic_keyPharse(pddl_line)
            extacted_commands.append(extacted_command)
        node,link=convert_PDDL_line_to_jsonGraph(extacted_commands)
        print(node)
        print("\n")
        print(link)
        post_data = {'message': LLM_message,
            'linkDataArray': link,
            'nodeDataArray': node
            }
        print("\njson")
        print(post_data)
        send_data_to_url(url_update_graph_and_chat,post_data)

        #This
        print("Extracted Commands:", extracted_commands)
        execute_subtasks(extracted_commands)

        return jsonify({'status': 'success'}), 200
    except Exception as e:
        print(f"Error: {e}")
        return jsonify({'error': 'An error occurred'}), 500

#thisss
def start_ros2():
    global ros2_initialized, arm1, arm2
    rclpy.init()
    ros2_initialized = True
    arm1 = ARM1_Skills()
    arm2 = ARM2_Skills()
    print("ROS 2 initialized with robot interfaces.")


if __name__ == "__main__":
   # start_main()
   ##thisss
    ros2_thread = threading.Thread(target=start_ros2)
    ros2_thread.daemon = True
    ros2_thread.start()
    app.run(port=5000, debug=True)

