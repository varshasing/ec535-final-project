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
from sensor_msgs.msg import LaserScan
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

# Gait config
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

BlockForward = False
BlockBackward = False
MoveForwardRequested = False
MoveBackwardRequested = False
SAFE_DISTANCE = 0.2  # meters

def lidar_callback(msg):
    global BlockForward, BlockBackward
    ranges = msg.ranges
    center = len(ranges) // 2

    # Front zone = center → blocks reverse
    front_window = ranges[center - 10:center + 10]
    # Rear zone = sides → blocks forward
    rear_window = ranges[:20] + ranges[-20:]

    # Clean valid readings
    front_valid = [r for r in front_window if 0.01 < r < float('inf')]
    rear_valid = [r for r in rear_window if 0.01 < r < float('inf')]

    BlockBackward = front_valid and min(front_valid) < SAFE_DISTANCE
    BlockForward = rear_valid and min(rear_valid) < SAFE_DISTANCE


def control_timer(event):
    if MoveForwardRequested and BlockForward:
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    elif MoveBackwardRequested and BlockBackward:
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)

# Cleanup on shutdown
def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('Shutting down.')

async def command_handler(websocket):
    global PuppyMove, MoveForwardRequested, MoveBackwardRequested
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
                ActionHandler.change_action_value("push-up.d6ac", 1)
            elif key == 'w':
                MoveForwardRequested = True
                MoveBackwardRequested = False
                if BlockForward:
                    rospy.logwarn("Blocked: Obstacle ahead, not moving forward.")
                    PuppyMove.update({'x': 0, 'y': 0, 'yaw_rate': 0})
                else:
                    PuppyMove.update({'x': 10, 'y': 0, 'yaw_rate': 0})
            elif key == 's':
                MoveForwardRequested = False
                MoveBackwardRequested = True
                if BlockBackward:
                    rospy.logwarn("Blocked: Obstacle behind, not moving backward.")
                    PuppyMove.update({'x': 0, 'y': 0, 'yaw_rate': 0})
                else:
                    PuppyMove.update({'x': -10, 'y': 0, 'yaw_rate': 0})
            elif key == 'a':
                MoveForwardRequested = False
                MoveBackwardRequested = False
                PuppyMove.update({'x': 2, 'y': 0, 'yaw_rate': 0.25})
            elif key == 'd':
                MoveForwardRequested = False
                MoveBackwardRequested = False
                PuppyMove.update({'x': 2, 'y': 0, 'yaw_rate': -0.25})
            elif key == 'x':
                MoveForwardRequested = False
                MoveBackwardRequested = False
                PuppyMove.update({'x': 0, 'y': 0, 'yaw_rate': 0})

            PuppyVelocityPub.publish(**PuppyMove)
        except json.JSONDecodeError:
            rospy.logwarn(f"Invalid JSON: {message}")

# WebSocket server runner
async def start_websocket_server():
    async with websockets.serve(command_handler, "0.0.0.0", 8765):
        await asyncio.Future()

def websocket_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_websocket_server())

# === MAIN ===
if __name__ == '__main__':
    print('init')

    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    print("python version:", sys.version)
    print('ros init')

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)

    # Start action group thread
    ActionHandler.start_action_thread()

    # Subscribe to LIDAR scan
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    # Add auto-block control timer
    rospy.Timer(rospy.Duration(0.1), control_timer)

    # Send initial pose/gait config
    rospy.sleep(0.2)
    PuppyPosePub.publish(
        stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500
    )
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(**GaitConfig)
    rospy.sleep(0.2)
    PuppyVelocityPub.publish(**PuppyMove)

    PuppyPosePub.publish(
        stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=-7, roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500
    )
    rospy.sleep(1)
    PuppyPosePub.publish(
        stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=-11, roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500
    )

    # Start WebSocket
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
