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
from collections import deque

ROS_NODE_NAME = 'puppy_control_server'

PuppyMove = {'x': 0, 'y': 0, 'yaw_rate': 0}
PuppyPose = {
    'roll': math.radians(0),
    'pitch': math.radians(0),
    'yaw': 0.000,
    'height': -10,
    'x_shift': -0.5,
    'stance_x': 0,
    'stance_y': 0
}

gait = 'Trot'
if gait == 'Trot':
    GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.3, 'clearance_time': 0.0, 'z_clearance': 5}
    PuppyPose['x_shift'] = -0.6

BlockForward = False
BlockBackward = False
MoveForwardRequested = False
MoveBackwardRequested = False
CurrentSpeed = 10.0
SAFE_DISTANCE = 0.3  # meters

WebSocketClients = set()
BroadcastQueue = deque()
BroadcastLock = threading.Lock()

def lidar_callback(msg):
    global BlockForward, BlockBackward
    ranges = msg.ranges
    center = len(ranges) // 2

    front_window = ranges[center - 10:center + 10]
    rear_window = ranges[:20] + ranges[-20:]

    front_valid = [r for r in front_window if 0.01 < r < float('inf')]
    rear_valid = [r for r in rear_window if 0.01 < r < float('inf')]

    BlockBackward = front_valid and min(front_valid) < SAFE_DISTANCE
    BlockForward = rear_valid and min(rear_valid) < SAFE_DISTANCE

    status = {"status": "blocked_front" if BlockForward else "blocked_back" if BlockBackward else "clear"}

    with BroadcastLock:
        BroadcastQueue.append(json.dumps(status))

def control_timer(event):
    if MoveForwardRequested and BlockForward:
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    elif MoveBackwardRequested and BlockBackward:
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)

def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('Shutting down.')

async def command_handler(websocket):
    global PuppyMove, MoveForwardRequested, MoveBackwardRequested, CurrentSpeed
    WebSocketClients.add(websocket)

    async for message in websocket:
        try:
            data = json.loads(message)
            key = data.get("key", "")
            height = data.get("height")
            speed = data.get("speed")

            if height is not None:
                PuppyPose['height'] = float(height)
                PuppyPosePub.publish(
                    stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                    height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'],
                    yaw=PuppyPose['yaw'], run_time=500
                )

            if speed is not None:
                CurrentSpeed = float(speed)
                if MoveForwardRequested and not BlockForward:
                    PuppyMove['x'] = CurrentSpeed
                    PuppyVelocityPub.publish(**PuppyMove)
                elif MoveBackwardRequested and not BlockBackward:
                    PuppyMove['x'] = -CurrentSpeed
                    PuppyVelocityPub.publish(**PuppyMove)

            rospy.loginfo(f"Received key command: {key}")
            if not ActionHandler.runningAction:
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
                        rospy.logwarn("Blocked: Obstacle ahead.")
                        PuppyMove.update({'x': 0, 'y': 0, 'yaw_rate': 0})
                    else:
                        PuppyMove.update({'x': CurrentSpeed, 'y': 0, 'yaw_rate': 0})
                elif key == 's':
                    MoveForwardRequested = False
                    MoveBackwardRequested = True
                    if BlockBackward:
                        rospy.logwarn("Blocked: Obstacle behind.")
                        PuppyMove.update({'x': 0, 'y': 0, 'yaw_rate': 0})
                    else:
                        PuppyMove.update({'x': -CurrentSpeed, 'y': 0, 'yaw_rate': 0})
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
                
                if 'type' in data and data['type'] == 'ping':
                    await websocket.send(json.dumps({
                        'type': 'pong',
                        'timestamp': data['timestamp']
                    }))
                
        except json.JSONDecodeError:
            rospy.logwarn(f"Invalid JSON: {message}")

async def broadcast_status_loop():
    while True:
        await asyncio.sleep(0.1)
        if BroadcastQueue:
            with BroadcastLock:
                while BroadcastQueue:
                    msg = BroadcastQueue.popleft()
                    dead = set()
                    for client in WebSocketClients:
                        try:
                            await client.send(msg)
                        except:
                            dead.add(client)
                    WebSocketClients.difference_update(dead)

async def start_websocket_server():
    server = await websockets.serve(command_handler, "0.0.0.0", 8765)
    await broadcast_status_loop()

def websocket_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_websocket_server())

if __name__ == '__main__':
    print('init')
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    print("connected")

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)

    ActionHandler.start_action_thread()
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Timer(rospy.Duration(0.1), control_timer)

    rospy.sleep(0.2)
    PuppyPosePub.publish(
        stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
        height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'],
        yaw=PuppyPose['yaw'], run_time=500
    )
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(**GaitConfig)
    rospy.sleep(0.2)
    PuppyVelocityPub.publish(**PuppyMove)

    threading.Thread(target=websocket_thread, daemon=True).start()

    while True:
        try:
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit(0)
        except:
            sys.exit(0)
