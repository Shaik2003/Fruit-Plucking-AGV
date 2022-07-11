'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 3 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ BM_1489 ]
# Author List:		[ Shaik Abdullah, Gurkirat Singh, Kaushik Vempati, Harika S]
# Filename:			task_3.py
# Functions:		heuristic(), _siftdown(), _siftdown_max(), heappush(), heappop()
# Global variables:	curr_cords
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os
import sys
import traceback
import math
import time
import sys
from pyzbar.pyzbar import decode

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
def heuristic(a, b):  # Calculates heuristic values for A* Path Planning
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def _siftdown(heap, startpos, pos):
    newitem = heap[pos]
    # Follow the path to the root, moving parents down until finding a place
    # newitem fits.
    while pos > startpos:
        parentpos = (pos - 1) >> 1
        parent = heap[parentpos]
        if newitem < parent:
            heap[pos] = parent
            pos = parentpos
            continue
        break
    heap[pos] = newitem


def _siftup(heap, pos):
    endpos = len(heap)
    startpos = pos
    newitem = heap[pos]
    # Bubble up the smaller child until hitting a leaf.
    childpos = 2*pos + 1    # leftmost child position
    while childpos < endpos:
        # Set childpos to index of smaller child.
        rightpos = childpos + 1
        if rightpos < endpos and not heap[childpos] < heap[rightpos]:
            childpos = rightpos
        # Move the smaller child up.
        heap[pos] = heap[childpos]
        pos = childpos
        childpos = 2*pos + 1
    # The leaf at pos is empty now.  Put newitem there, and bubble it up
    # to its final resting place (by sifting its parents down).
    heap[pos] = newitem
    _siftdown(heap, startpos, pos)


def _siftdown_max(heap, startpos, pos):
    'Maxheap variant of _siftdown'
    newitem = heap[pos]
    # Follow the path to the root, moving parents down until finding a place
    # newitem fits.
    while pos > startpos:
        parentpos = (pos - 1) >> 1
        parent = heap[parentpos]
        if parent < newitem:
            heap[pos] = parent
            pos = parentpos
            continue
        break
    heap[pos] = newitem


def heappush(heap, item):
    """Push item onto heap, maintaining the heap invariant."""
    heap.append(item)
    _siftdown(heap, 0, len(heap)-1)


def heappop(heap):
    """Pop the smallest item off the heap, maintaining the heap invariant."""
    lastelt = heap.pop()    # raises appropriate IndexError if heap is empty
    if heap:
        returnitem = heap[0]
        heap[0] = lastelt
        _siftup(heap, 0)
        return returnitem
    return lastelt


##############################################################


def init_remote_api_server():
    """
    Purpose:
    ---
    This function should first close any open connections and then start
    communication thread with server i.e. CoppeliaSim.

    Input Arguments:
    ---
    None

    Returns:
    ---
    `client_id` 	:  [ integer ]
            the client_id generated from start connection remote API, it should be stored in a global variable

    Example call:
    ---
    client_id = init_remote_api_server()

    """

    client_id = -1

    ##############	ADD YOUR CODE HERE	##############

    sim.simxFinish(-1)
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    ##################################################

    return client_id


def start_simulation(client_id):
    """
    Purpose:
    ---
    This function should first start the simulation if the connection to server
    i.e. CoppeliaSim was successful and then wait for last command sent to arrive
    at CoppeliaSim server end.

    Input Arguments:
    ---
    `client_id`    :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    `return_code` 	:  [ integer ]
            the return code generated from the start running simulation remote API

    Example call:
    ---
    return_code = start_simulation()

    """
    return_code = -2

    ##############	ADD YOUR CODE HERE	##############

    if client_id != -1:
        return_code = sim.simxStartSimulation(
            client_id, sim.simx_opmode_oneshot)

    ##################################################

    return return_code


def get_vision_sensor_image(client_id):
    """
    Purpose:
    ---
    This function should first get the handle of the Vision Sensor object from the scene.
    After that it should get the Vision Sensor's image array from the CoppeliaSim scene.
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    `vision_sensor_image` 	:  [ list ]
            the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
            the image resolution returned from the get vision sensor image remote API
    `return_code` 			:  [ integer ]
            the return code generated from the remote API

    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
    """

    return_code = 0

    ##############	ADD YOUR CODE HERE	##############
    vision_sensor_image = []
    image_resolution = []
    vision_sensor_return_code, vision_sensor_handle = sim.simxGetObjectHandle(
        client_id, "vision_sensor_1", sim.simx_opmode_oneshot_wait)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(
        client_id, vision_sensor_handle, 0, sim.simx_opmode_oneshot_wait)
    ##################################################

    return vision_sensor_image, image_resolution, return_code


def transform_vision_sensor_image(vision_sensor_image, image_resolution):
    """
    Purpose:
    ---
    This function should:
    1. First convert the vision_sensor_image list to a NumPy array with data-type as uint8.
    2. Since the image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
    the new NumPy array should then be resized to a 3-D (three dimensional) NumPy array.
    3. Change the color of the new image array from BGR to RGB.
    4. Flip the resultant image array about the X-axis.
    The resultant image NumPy array should be returned.

    Input Arguments:
    ---
    `vision_sensor_image` 	:  [ list ]
            the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
            the image resolution returned from the get vision sensor image remote API

    Returns:
    ---
    `transformed_image` 	:  [ numpy array ]
            the resultant transformed image array after performing above 4 steps

    Example call:
    ---
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)

    """

    transformed_image = None

    ##############	ADD YOUR CODE HERE	##############

    transformed_image = np.array(vision_sensor_image)
    transformed_image = transformed_image.astype(np.uint8)
    # print(transformed_image.shape)
    transformed_image = transformed_image.reshape(
        image_resolution[0], image_resolution[1], 3)
    # print(transformed_image.shape)
    transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)
    transformed_image = cv2.flip(transformed_image, 1)

    ##################################################

    return transformed_image


def stop_simulation(client_id):
    """
    Purpose:
    ---
    This function should stop the running simulation in CoppeliaSim server.
    NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
              It is already written in the main function.

    Input Arguments:
    ---
    `client_id`    :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    `return_code` 	:  [ integer ]
            the return code generated from the stop running simulation remote API

    Example call:
    ---
    return_code = stop_simulation()

    """

    return_code = -2

    ##############	ADD YOUR CODE HERE	##############

    if return_code != -1:
        return_code = sim.simxStopSimulation(
            client_id, sim.simx_opmode_oneshot_wait)

    ##################################################

    return return_code


def exit_remote_api_server(client_id):
    """
    Purpose:
    ---
    This function should wait for the last command sent to arrive at the Coppeliasim server
    before closing the connection and then end the communication thread with server
    i.e. CoppeliaSim using simxFinish Remote API.
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    None

    Example call:
    ---
    exit_remote_api_server()

    """

    ##############	ADD YOUR CODE HERE	##############

    sim.simxGetPingTime(client_id)
    sim.simxFinish(client_id)

    ##################################################


def detect_qr_codes(transformed_image):
    """
    Purpose:
    ---
    This function receives the transformed image from the vision sensor and detects qr codes in the image

    Input Arguments:
    ---
    `transformed_image` 	:  [ numpy array ]
            the transformed image array

    Returns:
    ---
    None

    Example call:
    ---
    detect_qr_codes()

    """

    ##############	ADD YOUR CODE HERE	##############
    qr_code = 'hi'
    qr_codes = []
    qr_codes = decode(transformed_image)
    # print(transformed_image.shape)
    for qr_code in qr_codes:
        # qr_data = []
        # print(qr_code.data.decode("utf-8"))
        # if qr_code.data.decode("utf-8") == "(0, 0)":
        # 	print("hello")
        qr_code = qr_code.data.decode("utf-8")
    if qr_code != 'hi':
        qr_codes = [float(item) for item in qr_code[1:-1].split(", ")]
    else:
        qr_codes = qr_code
    ##################################################

    return qr_codes


def set_bot_movement(client_id, wheel_joints, forw_back_vel, left_right_vel, rot_vel):
    """
    Purpose:
    ---
    This function takes desired forward/back, left/right, rotational velocites of the bot as input arguments.
    It should then convert these desired velocities into individual joint velocities(4 joints) and actuate the joints
    accordingly.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    'wheel_joints`      :   [ list]
            Python list containing joint object handles of individual joints

    `forw_back_vel'     :   [ float ]
            Desired forward/back velocity of the bot

    `left_right_vel'    :   [ float ]
            Desired left/back velocity of the bot

    `rot_vel'           :   [ float ]
            Desired rotational velocity of the bot

    Returns:
    ---
    None

    Example call:
    ---
    set_bot_movement(client_id, wheel_joints, 0.5, 0, 0)

    """

    ##############	ADD YOUR CODE HERE	##############
    separation_width = 0.3170/2
    separation_length = 0.4760/2
    radius = 0.05
    vel_fl = (1/radius)*(forw_back_vel+left_right_vel -
                         (separation_width+separation_length)*rot_vel)
    vel_fr = (1/radius)*(forw_back_vel-left_right_vel +
                         (separation_width+separation_length)*rot_vel)
    vel_rl = (1/radius)*(forw_back_vel-left_right_vel -
                         (separation_width+separation_length)*rot_vel)
    vel_rr = (1/radius)*(forw_back_vel+left_right_vel +
                         (separation_width+separation_length)*rot_vel)
    return_code = sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[0], vel_fl, sim.simx_opmode_oneshot_wait)
    return_code = sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[1], vel_fr, sim.simx_opmode_oneshot_wait)
    return_code = sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[2], vel_rl, sim.simx_opmode_oneshot_wait)
    return_code = sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[3], vel_rr, sim.simx_opmode_oneshot_wait)

    ##################################################


def init_setup(client_id):
    """
    Purpose:
    ---
    This function will get the object handles of all the four joints in the bot, store them in a list
    and return the list

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    'wheel_joints`      :   [ list]
            Python list containing joint object handles of individual joints

    Example call:
    ---
    init setup(client_id)

    """
    wheel_joints = []
    # fl fr rl rr
    ##############	ADD YOUR CODE HERE	##############
    # fl
    return_code, obj_handle = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_fl', sim.simx_opmode_oneshot_wait)
    wheel_joints.append(obj_handle)
    # fr
    return_code, obj_handle = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_fr', sim.simx_opmode_oneshot_wait)
    wheel_joints.append(obj_handle)
    # rl
    return_code, obj_handle = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_rl', sim.simx_opmode_oneshot_wait)
    wheel_joints.append(obj_handle)
    # rr
    return_code, obj_handle = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_rr', sim.simx_opmode_oneshot_wait)
    wheel_joints.append(obj_handle)
    ##################################################

    return wheel_joints


def encoders(client_id):
    """
    Purpose:
    ---
    This function will get the `combined_joint_position` string signal from CoppeliaSim, decode it
    and return a list which contains the total joint position of all joints    

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    'joints_position`      :   [ list]
            Python list containing the total joint position of all joints

    Example call:
    ---
    encoders(client_id)

    """

    return_code, signal_value = sim.simxGetStringSignal(
        client_id, 'combined_joint_position', sim.simx_opmode_blocking)
    signal_value = signal_value.decode()
    joints_position = signal_value.split("%")

    for index, joint_val in enumerate(joints_position):
        joints_position[index] = float(joint_val)

    return joints_position


def nav_logic(client_id, qr_code, wheel_joints, start, target_points, transformed_image):
    """
    Purpose:
    ---
    This function should implement your navigation logic. 
    """
    global curr_cords
    if qr_code != 'hi':
        curr_cords = qr_code
    for point in target_points:
        route = shortest_path(start, point)

        route = route + [start]
        route = route[::-1]
        # print(route)
        for point in route:
            final_cords = point
            ex = final_cords[0] - curr_cords[0]
            ey = final_cords[1] - curr_cords[1]
            k = 0.25
            # print(route)
            # print(point)
            if point == route[len(route)-1]:
                k = 0.1
            vx = k*ex
            vy = k*ey
            omega = 0
            set_bot_movement(client_id, wheel_joints, vy, vx, omega)
            while point != tuple(qr_code):
                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                qr_code = detect_qr_codes(transformed_image)
            if point == tuple(qr_code):
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            curr_cords = list(point)
        start = point
        if target_points[-1] == point:
            break


def shortest_path(start, goal):
    """
    Purpose:
    ---
    This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
    """

    array = np.zeros((12, 12))

    array[3][3] = 1
    array[3][4] = 1
    possible_directions = [(0, 1), (0, -1), (1, 0),
                           (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    heap = []
    heappush(heap, (fscore[start], start))
    while heap:
        current = heappop(heap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)
        for i, j in possible_directions:
            possible_direction = current[0] + i, current[1] + j
            g_dash = gscore[current] + heuristic(current, possible_direction)
            if 0 <= possible_direction[0] < array.shape[0]:
                if 0 <= possible_direction[1] < array.shape[1]:
                    if array[possible_direction[0]][possible_direction[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
            if possible_direction in close_set and g_dash >= gscore.get(possible_direction, 0):
                continue
            if g_dash < gscore.get(possible_direction, 0) or possible_direction not in [i[1]for i in heap]:
                came_from[possible_direction] = current
                gscore[possible_direction] = g_dash
                fscore[possible_direction] = g_dash + \
                    heuristic(possible_direction, goal)
                heappush(
                    heap, (fscore[possible_direction], possible_direction))
    return False


def task_3_primary(client_id, target_points):
    """
    Purpose:
    ---

    # NOTE:This is the only function that is called from the main function and from the executable.

    Make sure to call all the necessary functions (apart from the ones called in the main) according to your logic. 
    The bot should traverses all the target navigational co-ordinates.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    `target_points`     : [ list ]
            List of tuples where tuples are the target navigational co-ordinates.

    Returns:
    ---

    Example call:
    ---
    target_points(client_id, target_points)

    """

    wheel_joints = init_setup(client_id)
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
        client_id)
    transformed_image = transform_vision_sensor_image(
        vision_sensor_image, image_resolution)
    qr_code = detect_qr_codes(transformed_image)

    nav_logic(client_id, qr_code, wheel_joints,
              (0, 0), target_points, transformed_image)


if __name__ == "__main__":

    ##################################################
    # target_points is a list of tuples. These tuples are the target navigational co-ordinates
    # target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
    # example:
    # You can give any number of different co-ordinates
    target_points = [(2, 3), (3, 6), (8, 11), (0, 0)]

    ##################################################
    ## NOTE: You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')

    try:
        client_id = init_remote_api_server()
        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')

            # Starting the Simulation
            try:
                return_code = start_simulation(client_id)

                if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
                    print('\nSimulation started correctly in CoppeliaSim.')

                else:
                    print(
                        '\n[ERROR] Failed starting the simulation in CoppeliaSim!')
                    print(
                        'start_simulation function is not configured correctly, check the code!')
                    print()
                    sys.exit()

            except Exception:
                print(
                    '\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
                print('Stop the CoppeliaSim simulation manually.\n')
                traceback.print_exc(file=sys.stdout)
                print()
                sys.exit()

        else:
            print('\n[ERROR] Failed connecting to Remote API server!')
            print('[WARNING] Make sure the CoppeliaSim software is running and')
            print(
                '[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
            print(
                '[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
            print()
            sys.exit()

    except Exception:
        print(
            '\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    try:

        task_3_primary(client_id, target_points)
        time.sleep(1)

        try:
            return_code = stop_simulation(client_id)
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.')

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    exit_remote_api_server(client_id)
                    if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
                        print(
                            '\nDisconnected successfully from Remote API Server in CoppeliaSim!')

                    else:
                        print(
                            '\n[ERROR] Failed disconnecting from Remote API server!')
                        print(
                            '[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

                except Exception:
                    print(
                        '\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print(
                    '\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
                print(
                    '[ERROR] stop_simulation function is not configured correctly, check the code!')
                print('Stop the CoppeliaSim simulation manually.')

            print()
            sys.exit()

        except Exception:
            print(
                '\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    except Exception:
        print(
            '\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()
