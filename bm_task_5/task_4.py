'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ BM-1489 ]
# Author List:		[ Shaik, Gurkirat, Kaushik, Harika ]
# Filename:			task_4.py
# Functions:
# Global variables:
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
from pyzbar.pyzbar import decode
#############################################################
#############################################################
import task_1b
import task_2a
import task_3
##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()


# ADD UTILITY FUNCTIONS HERE #################+
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def trajectory(client_id, current, target):

    # y = ((0.74)**2-current[0]**2-current[2]**2)**0.5
    # current = [current[0], y, current[2]]
    # reachTarget(client_id, current)
    current_copy = current.copy()
    target_copy = target.copy()
    step_size = 0.05
    # time.sleep(sleep_time)
    while True:
        error = np.array(target_copy)-np.array(current_copy)

        for i in range(3):
            if error[i] >= step_size:
                current_copy[i] += step_size
            elif -1*error[i] >= step_size:
                current_copy[i] -= step_size
        if abs(error[0]) < step_size and abs(error[1]) < step_size and abs(error[2]) < step_size:
            break
        time.sleep(0.075)
        return_code, handle_prox_sensor = sim.simxGetObjectHandle(
            client_id, 'RG2_attachProxSensor', sim.simx_opmode_blocking)
        return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(
            client_id, handle_prox_sensor, sim.simx_opmode_oneshot_wait)
        # print(detection_state, detected_point,
        #       detected_object_handle, detected_surface_normal_vector)

    reachTarget(client_id, target_copy)
    print(detection_state, detected_point,
          detected_object_handle, detected_surface_normal_vector)


def call_open_close(client_id, command):
    command = [command]
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'gripper', sim.sim_scripttype_childscript, 'open_close', [], [], command, emptybuff, sim.simx_opmode_blocking)
    # print(command)


def reachTarget(client_id, coor):
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'robotic_arm', sim.sim_scripttype_childscript, 'setPosition', [], coor, [], emptybuff, sim.simx_opmode_blocking)
    print(return_code, outints, oufloats, outstring, outbuffer)

##############################################################


def send_identified_berry_data(client_id, berry_name, x_coor, y_coor, depth):
    """
    Purpose:
    ---
    Teams should call this function as soon as they identify a berry to pluck. This function should be called only when running via executable.

    NOTE: 
    1. 	Correct Pluck marks will only be awarded if the team plucks the last detected berry. 
            Hence before plucking, the correct berry should be identified and sent via this function.

    2.	Accuracy of detection should be +-0.025m.

    Input Arguments:
    ---
    `client_id` 	:  [ integer ]
            the client_id generated from start connection remote API, it should be stored in a global variable

    'berry_name'		:	[ string ]
                    name of the detected berry.

    'x_coor'			:	[ float ]
                    x-coordinate of the centroid of the detected berry.

    'y_coor'			:	[ float ]
                    y-coordinate of the centroid of the detected berry.

    'depth'			:	[ float ]
                    z-coordinate of the centroid of the detected berry.

    Returns:
    ---
    `return_code`		:	[ integer ]
                    A remote API function return code
                    https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes

    Example call:
    ---
    return_code=send_identified_berry_data(berry_name,x_coor,y_coor)

    """
    ##################################################
    ## You are NOT allowed to make any changes in the code below. ##
    emptybuff = bytearray()

    if(type(berry_name) != str):
        berry_name = str(berry_name)

    if(type(x_coor) != float):
        x_coor = float(x_coor)

    if(type(y_coor) != float):
        y_coor = float(y_coor)

    if(type(depth) != float):
        depth = float(depth)

    data_to_send = [berry_name, str(x_coor), str(y_coor), str(depth)]
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'eval_bm', sim.sim_scripttype_childscript, 'detected_berry_by_team', [], [], data_to_send, emptybuff, sim.simx_opmode_blocking)
    return return_code

    ##################################################


def task_4_primary(client_id):
    # """
    # Purpose:
    # ---
    # This is the only function that is called from the main function. Make sure to fill it
    # properly, such that the bot traverses to the vertical rack, detects, plucks & deposits a berry of each color.

    # Input Arguments:
    # ---
    # `client_id`         :   [ integer ]
    # 	the client id of the communication thread returned by init_remote_api_server()

    # Returns:
    # ---

    # Example call:
    # ---
    # task_4_primary(client_id)

    # """

    return_code, handle_prox_sensor = sim.simxGetObjectHandle(
        client_id, 'RG2_attachProxSensor', sim.simx_opmode_blocking)

    wheel_joints = task_3.init_setup(client_id)
    vision_sensor_image1, image_resolution1, return_code1 = task_3.get_vision_sensor_image(
        client_id)
    transformed_image1 = task_3.transform_vision_sensor_image(
        vision_sensor_image1, image_resolution1)
    qr_code = task_3.detect_qr_codes(transformed_image1)
    target_points = [(4, 4)]
    print(qr_code)
    task_3.nav_logic(client_id, qr_code, wheel_joints, (int(qr_code[0]), int(qr_code[1])),
                     target_points, transformed_image1)

    return_code, vision_sensor2_handle = sim.simxGetObjectHandle(
        client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(
        client_id, vision_sensor2_handle)
    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(
        client_id, vision_sensor2_handle)
    transformed_image = task_1b.transform_vision_sensor_image(
        vision_sensor_image, image_resolution)
    transformed_depth_image = task_2a.transform_vision_sensor_depth_image(
        vision_sensor_depth_image, depth_image_resolution)
    berries_dictionary = task_2a.detect_berries(
        transformed_image, transformed_depth_image)
    berry_positions_dictionary = task_2a.detect_berry_positions(
        berries_dictionary)
    # print(berry_positions_dictionary)
    box = [-0.55, 0.1, -0.15]
    default_pos = [0, -0.74, 0]
    sleep_time = 0.5
    close_time = 0.25

    ####################################################################################
    berry_positions_dictionary["Strawberry"].sort(key=lambda x: x[2])

    for i in range(30):
        call_open_close(client_id, "open")

    time.sleep(close_time)

    x, y, z = berry_positions_dictionary["Strawberry"][0]

    return_code = send_identified_berry_data(client_id, 'Strawberry', x, y, z)

    coor = [x, y, z]
    trajectory(client_id, default_pos, coor)
    time.sleep(sleep_time)
    return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(
        client_id, handle_prox_sensor, sim.simx_opmode_oneshot_wait)
    print(detection_state, detected_point,
          detected_object_handle, detected_surface_normal_vector)
    for i in range(30):
        call_open_close(client_id, "close")
    # time.sleep(close_time)

    # print(default_pos)
    trajectory(client_id, coor, default_pos)
    # print("###########default########################################")
    # time.sleep(1)
    trajectory(client_id, default_pos, box)
    time.sleep(close_time)

    for i in range(30):
        call_open_close(client_id, "open")
    time.sleep(close_time)
    for i in range(30):
        call_open_close(client_id, "close")
    trajectory(client_id, box, default_pos)
    ##################.########################
    ##############################################################################################################################
    berry_positions_dictionary["Lemon"].sort(key=lambda x: x[2])
    # print(sorted, berry_positions_dictionary["Lemon"])

    for i in range(30):
        call_open_close(client_id, "open")
    coor = []

    x, y, z = berry_positions_dictionary["Lemon"][0]

    return_code = send_identified_berry_data(client_id, 'Lemon', x, y, z)

    coor = [x, y, z]
    # time.sleep(1)
    trajectory(client_id, default_pos, coor)
    # reachTarget(client_id, coor)
    # print(coor)
    time.sleep(close_time)
    for i in range(30):
        call_open_close(client_id, "close")

    # time.sleep(close_time)
    # reachTarget(client_id,default_pos)
    trajectory(client_id, coor, default_pos)
    # time.sleep(0.3)
    # time.sleep(1)
    trajectory(client_id, default_pos, box)
    time.sleep(close_time)
    for i in range(30):
        call_open_close(client_id, "open")
    time.sleep(close_time)
    for i in range(30):
        call_open_close(client_id, "close")
    trajectory(client_id, box, default_pos)
#####################################
# reachTarget(client_id,default_pos)
    # time.sleep(sleep_time)

    #########################################

    berry_positions_dictionary["Blueberry"].sort(key=lambda x: x[2])

    for i in range(20):
        call_open_close(client_id, "open")

    x, y, z = berry_positions_dictionary["Blueberry"][0]

    return_code = send_identified_berry_data(client_id, 'Blueberry', x, y, z)

    coor = [x, y, z]
    # reachTarget(client_id, [x,y,0])
    # time.sleep(1)
    #reachTarget(client_id, coor)
    trajectory(client_id, default_pos, coor)
    # print(coor)
    time.sleep(sleep_time)
    for i in range(30):
        call_open_close(client_id, "close")
    # time.sleep(close_time)
    #reachTarget(client_id, [x,y,0])
    # time.sleep(1)

    # time.sleep(1)
    # reachTarget(client_id,default_pos)
    trajectory(client_id, coor, default_pos)
    # time.sleep(0.2)

    trajectory(client_id, default_pos, box)
    time.sleep(sleep_time)
    for i in range(20):
        call_open_close(client_id, "open")
    trajectory(client_id, box, default_pos)
    #########################################


if __name__ == "__main__":

    ##################################################
    ## You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')

    try:
        client_id = task_1b.init_remote_api_server()
        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')

            # Starting the Simulation
            try:
                return_code = task_1b.start_simulation(client_id)

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

        task_4_primary(client_id)
        time.sleep(1)

        try:
            return_code = task_1b.stop_simulation(client_id)
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.')

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    task_1b.exit_remote_api_server(client_id)
                    if (task_1b.start_simulation(client_id) == sim.simx_return_initialize_error_flag):
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
            '\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()
