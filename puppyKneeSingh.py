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
    async for message in websocket:
        try:
            data = json.loads(message)
            key = data.get("key", "")
            rospy.loginfo(f"Received key command: {key}")

            if key == 'w':
                PuppyMove.update({'x': 10, 'y': 0, 'yaw_rate': 0})
            elif key == 's':
                PuppyMove.update({'x': -10, 'y': 0, 'yaw_rate': 0})
            elif key == 'a':
                PuppyMove.update({'x': 2, 'y': 0, 'yaw_rate': 0.18})
            elif key == 'd':
                PuppyMove.update({'x': 2, 'y': 0, 'yaw_rate': -0.18})
            elif key == 'x':
                PuppyMove.update({'x': 0, 'y': 0, 'yaw_rate': 0})
                PuppyGaitConfigPub.publish(**GaitConfig)

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

    # ROS publishers and services
    print('ros')
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)

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
    set_mark_time_srv(True)

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
