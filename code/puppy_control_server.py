#!/usr/bin/env python3
# coding=utf8

import sys
import json 
import math
import rospy
import asyncio
import threading
import websockets
from std_srvs.srv import SetBool
from puppy_control.msg import Velocity, Pose, Gait
import action_test as ActionHandler
ROS_NODE_NAME = 'puppy_control_server'


# Default walking velocity
PuppyMove = {'x': 0, 'y': 0, 'yaw_rate': 0}

# Default pose
PuppyPose = {
    'roll': math.radians(0),
    'pitch': math.radians(0),
    'yaw': 0.000,
    'height': -10,
    'x_shift': -0.5,
    'stance_x': 0,
    'stance_y': 0
}
# may need to edit this for the different speeds ?
# Default gait config
gait = 'Trot'
if gait == 'Trot':
    GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.3, 'clearance_time': 0.0, 'z_clearance': 5}
    PuppyPose['x_shift'] = -0.6
elif gait == 'Amble':
    GaitConfig = {'overlap_time': 0.1, 'swing_time': 0.2, 'clearance_time': 0.1, 'z_clearance': 5}
    PuppyPose['x_shift'] = -0.9
elif gait == 'Walk':
    GaitConfig = {'overlap_time': 0.1, 'swing_time': 0.2, 'clearance_time': 0.3, 'z_clearance': 5}
    PuppyPose['x_shift'] = -0.65

def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('Shutting down.')

# WebSocket command handling
async def command_handler(websocket):
    global PuppyMove
    global set_mark_time_srv
    async for message in websocket:
        try:
            data = json.loads(message)
            key = data.get("key", "")
            rospy.loginfo(f"Received key command: {key}")
            

            if key == 'bow':
                ActionHandler.change_action_value("bow.d6ac", 1)
            elif key == "lie_down":
                ActionHandler.change_action_value("lie_down.d6ac", 1)
            elif key == "moonwalk":
                ActionHandler.change_action_value("moonwalk.d6ac", 1)
            elif key == "nod":
                ActionHandler.change_action_value("nod.d6ac", 1)
            elif key == "spacewalk":
                ActionHandler.change_action_value("spacewalk.d6ac", 1)
            elif key == "stand":
                ActionHandler.change_action_value("stand.d6ac", 1)
            elif key == "push-up":
                Actionhandler.change_action_value("push-up.d6ac", 1)
            elif key == 'w':
                PuppyMove.update({'x': 10, 'y': 0, 'yaw_rate': 0})
            elif key == 's':
                PuppyMove.update({'x': -10, 'y': 0, 'yaw_rate': 0})
            elif key == 'a':
                PuppyMove.update({'x': 2, 'y': 0, 'yaw_rate': 0.25})
            elif key == 'd':
                PuppyMove.update({'x': 2, 'y': 0, 'yaw_rate': -0.25})
            elif key == 'x':
                PuppyMove.update({'x': 0, 'y': 0, 'yaw_rate': 0})
                
                """PuppyVelocityPub.publish(**PuppyMove)
                
                # Define a blocking function wrapper
                def call_mark_time():
                    try:
                        resp = set_mark_time_srv(True)
                        rospy.loginfo(f"Set mark time success: {resp.success}, message: {resp.message}")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Failed to call set_mark_time_srv: {e}")

                # Run it in a background thread
                #loop = asyncio.get_event_loop()
                #await loop.run_in_executor(None, call_mark_time) 
                #loop = asyncio.get_event_loop()
                #await loop.run_in_executor(None, set_mark_time_srv, True)"""

            PuppyVelocityPub.publish(**PuppyMove)
        except json.JSONDecodeError:
            rospy.logwarn(f"Invalid JSON: {message}")

async def start_websocket_server():
    async with websockets.serve(command_handler, "0.0.0.0", 8765):
        await asyncio.Future()

def websocket_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_websocket_server())

if __name__ == '__main__':
    print('init')
    
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    print("python version:", sys.version)
    # ROS publishers and services
    print('ros')
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)
    
    # start acction group thread
    ActionHandler.start_action_thread()
    
    
    # Send initial setup messages
    rospy.sleep(0.2)
    PuppyPosePub.publish(
        stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500
    )
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(**GaitConfig)
    rospy.sleep(0.2)
    PuppyVelocityPub.publish(**PuppyMove)
    #set_mark_time_srv(True)
    
    PuppyPosePub.publish(
        stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=-7, roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500
    )
    
    rospy.sleep(1)
    
    PuppyPosePub.publish(
        stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=-11, roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500
    )

    # Start WebSocket in background
    print('start thread')
    threading.Thread(target=websocket_thread, daemon=True).start()
    print("connected")

    while True:
        try:
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit(0)
        except:
            sys.exit(0)

