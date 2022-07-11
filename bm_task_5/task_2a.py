'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 2A of Berryminator(BM) Theme (eYRC 2021-22).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*
*****************************************************************************************
'''

# Team ID:			[ 1489 ]
# Author List:		[ Shaik Abdullah, Gurkirat Singh, Kaushik Vempati, Harika S ]
# Filename:			task_2a.py
# Functions:
# Global variables:
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
import sys
import traceback
##############################################################

try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()

try:
    import task_1b

except ImportError:
    print('\n[ERROR] task_1b.py file is not present in the current directory.')
    print('Your current directory is: ', os.getcwd())
    print('Make sure task_1b.py is present in this current directory.\n')
    sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def findfruit(transformed_depth_image, transformed_image_HSV,
              weaker, stronger):
    """
    Purpose:
    ---
    Finding the pixel coordinates and depth from depth image of an  object of a particular colour and shape in an image.
    Input Arguments:
    ---
    `transformed_depth_image`    :   [ list ]
            depth image of the scene`
    `transformed_image_HSV`    :   [ integer ]
            image converted into hsv format
    `weaker`    :   [ integer ]
            lower limit of HSV
    `stronger`    :   [ integer ]
            upper limit of HSv
    Returns:
    ---
    `coordinates` 	:  [ Tuple ]
            pixel coordinates and depth of the object

    Example call:
    ---
    coordinates_lemon = findfruit(transformed_depth_image, transformed_image_HSV, lemon_weaker, lemon_stronger)
    """
    mask = cv2.inRange(
        transformed_image_HSV, weaker, stronger)
    coordinates = []
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for contour in contours:

        M = cv2.moments(contour)
        cX = int(M["m10"]/M["m00"])
        cY = int(M["m01"]/M["m00"])

        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02*peri, True)
        if len(approx) > 5:

            coordinates.append((cX, cY, transformed_depth_image[cY][cX]))
    return coordinates

##############################################################


def get_vision_sensor_image(client_id, vision_sensor_handle):
    """
    Purpose:
    ---
    This function takes the client id and handle of the vision sensor scene object as input
    arguments and returns the vision sensor's image array from the CoppeliaSim scene.

    Input Arguments:
    ---
    `client_id`    :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()
    `vision_sensor_handle`    :   [ integer ]
            the handle of the vision sensor scene object

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

    vision_sensor_image = []
    image_resolution = []
    return_code = 0

    ##############	ADD YOUR CODE HERE	##############
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(
        client_id, vision_sensor_handle, 0, sim.simx_opmode_oneshot_wait)

    ##################################################

    return vision_sensor_image, image_resolution, return_code


def get_vision_sensor_depth_image(client_id, vision_sensor_handle):
    """
    Purpose:
    ---
    This function takes the client id and handle of the vision sensor scene object as input
    arguments and returns the vision sensor's depth buffer array from the CoppeliaSim scene.
    Input Arguments:
    ---
    `client_id`               :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()
    `vision_sensor_handle`    :   [ integer ]
            the handle of the vision sensor scene object

    Returns:
    ---
    `vision_sensor_depth_image` 	:  [ list ]
            the depth buffer array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
            the image resolution returned from the get vision sensor image remote API
    `return_code` 			:  [ integer ]
            the return code generated from the remote API

    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
    """

    vision_sensor_depth_image = []
    image_resolution = []
    return_code = 0

    ##############	ADD YOUR CODE HERE	##############
    return_code, image_resolution, vision_sensor_depth_image = sim.simxGetVisionSensorDepthBuffer(
        client_id, vision_sensor_handle, sim.simx_opmode_oneshot_wait)

    ##################################################

    return vision_sensor_depth_image, image_resolution, return_code


def transform_vision_sensor_depth_image(vision_sensor_depth_image, image_resolution):
    """
    Purpose:
    ---
    This function converts the depth buffer array received from vision sensor and converts it into
    a numpy array that can be processed by OpenCV
    This function should:
    1. First convert the vision_sensor_depth_image list to a NumPy array with data-type as float32.
    2. Since the depth image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
    the new NumPy array should then be resized to a 2-D (two dimensional) NumPy array.
    3. Flip the resultant image array about the appropriate axis. The resultant image NumPy array should be returned.

    Input Arguments:
    ---
    `vision_sensor_depth_image` 	:  [ list ]
            the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
            the image resolution returned from the get_vision_sensor_depth_image() function

    Returns:
    ---
    `transformed_depth_image` 	:  [ numpy GetObarray ]
            the resultant transformed image array after performing above 3 steps

    Example call:
    ---
    transformed_image = transform_vision_sensor_image(
        vision_sensor_image, image_resolution)

    """

    transformed_depth_image = None

    ##############	ADD YOUR CODE HERE	##############
    # print(image_resolution)
    transformed_depth_image = np.array(vision_sensor_depth_image)
    transformed_depth_image = transformed_depth_image.astype(np.float32)
    # print(transformed_image.shape)
    transformed_depth_image = transformed_depth_image.reshape(
        image_resolution[0], image_resolution[1])
    # print(transformed_image.shape)
    transformed_depth_image = cv2.flip(transformed_depth_image, 1)

    ##################################################

    return transformed_depth_image


def detect_berries(transformed_image, transformed_depth_image):
    """
    Purpose:
    ---
    This function takes the transformed image and transformed depth image as input arguments and returns
    the pixel coordinates and depth values in form of a dictionary.

    Input Arguments:
    ---
    `transformed_image` 	:  [ numpy array ]
            the transformed image array
    `transformed_depth_image` 	:  [ numpy array ]
            the transformed depth image array

    Returns:
    ---
    `berries_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of all the berries

    Example call:
    ---
    berries_dictionary = detect_berries(
        transformed_image, transformed_depth_image)

    """
    berries_dictionary = {}
    berries = ["Strawberry", "Blueberry", "Lemon"]

    ##############	ADD YOUR CODE HERE	##############

    transformed_image_HSV = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2HSV)

    ##################
    # range
    strawberry_weaker = np.array([138, 49, 55])
    strawberry_stronger = np.array([179, 255, 255])
    blueberry_stronger = np.array([130, 255, 255])
    blueberry_weaker = np.array([96, 50, 50])
    lemon_stronger = np.array([46, 255, 255])
    lemon_weaker = np.array([27, 70, 130])
    ##################
    #####################
    # find strawberry
    coordinates_strawberry = findfruit(transformed_depth_image,
                                       transformed_image_HSV, strawberry_weaker, strawberry_stronger)
    ###############
    coordinates_blueberry = findfruit(transformed_depth_image,
                                      transformed_image_HSV, blueberry_weaker, blueberry_stronger)
    #################
    coordinates_lemon = findfruit(transformed_depth_image, transformed_image_HSV,
                                  lemon_weaker, lemon_stronger)
    coordinates = []

    coordinates.append(coordinates_strawberry)
    coordinates.append(coordinates_blueberry)
    coordinates.append(coordinates_lemon)

    for i in range(len(berries)):
        berries_dictionary[berries[i]] = coordinates[i]

    ##################################################
    return berries_dictionary


def detect_berry_positions(berries_dictionary):
    """
    Purpose:
    ---
    This function takes the berries_dictionary as input arguments and calculates the 3D positions of the
    berries with respect to vision sensor. The final output is returned in another dictionary.

    Input Arguments:
    ---
    `berries_dictionary` 	:  [ dictionary ]
            the dictionary returned by detect_berries() function

    Returns:
    ---
    `berry_positions_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of 3D positions of all the berries

    Example call:
    ---
    berry_positions_dictionary = detect_berry_positions(berries_dictionary)

    """
    berry_positions_dictionary = {}
    berries = ["Strawberry", "Blueberry", "Lemon"]

    ##############	ADD YOUR CODE HERE	##############
    nearClippingPlane = 0
    farClippingPlane = 2
    for berry in berries:
        list_of_berry = []
        for fruit in berries_dictionary[berry]:
            (cX, cY, depth) = fruit
            error = 0.03
            distanceInMeters = nearClippingPlane + \
                depth * \
                (farClippingPlane-nearClippingPlane)
            list_of_berry.append(
                (np.round((cX-256)/512, 2), np.round((cY-256)/512, 2), np.round(distanceInMeters, 2)+error))
        berry_positions_dictionary[berry] = list_of_berry
    ##################################################

    return berry_positions_dictionary


def get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary):
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
    """
    Purpose:
    ---
    This function takes the transformed_image and the dictionaries returned by detect_berries()
    and  detect_berry_positions() functions. This function is already completed for your reference
    and will be helpful for debugging purposes.

    Input Arguments:
    ---
    `transformed_image` :	[ numpy array ]
                    numpy array of image returned by cv2 library

    `berries_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of all the berries

    `berry_positions_dictionary` 	:  [ dictionary ]
            the resultant dictionary with details of 3D positions of all the berries

    Returns:
    ---
    `labelled_image` :	[ numpy array ]
                    labelled image

    Example call:
    ---
    transformed_image = get_labeled_image(
        transformed_image, berries_dictionary, berry_positions_dictionary)
    """
    labelled_image = np.array(transformed_image)
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########

    for berry_type in berries_dictionary.keys():
        berry_details_list = berries_dictionary[berry_type]
        berry_positions_list = berry_positions_dictionary[berry_type]
        for index in range(len(berry_details_list)):
            pixel_x, pixel_y, depth_val = berry_details_list[index]
            coordinates = (pixel_x, pixel_y)
            horizontal_displacement, vertical_displacement, distance_from_sensor = berry_positions_list[
                index]
            horizontal_displacement, vertical_displacement, distance_from_sensor = round(
                horizontal_displacement, 2), round(vertical_displacement, 2), round(distance_from_sensor, 2)
            cv2.putText(labelled_image, str((horizontal_displacement, vertical_displacement,
                        distance_from_sensor)), coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return labelled_image


if __name__ == "__main__":

    berries_dictionary = {}
    berry_positions_dictionary = {}

    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')
    cv2.namedWindow('transformed image', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('transformed depth image', cv2.WINDOW_AUTOSIZE)

    try:
        # Initiate Remote API connection
        client_id = task_1b.init_remote_api_server()

        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')
            return_code, vision_sensor_handle = sim.simxGetObjectHandle(
                client_id, 'vision_sensor', sim.simx_opmode_blocking)

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

    while True:

        # Get image array and depth buffer from vision sensor in CoppeliaSim scene
        try:
            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id, vision_sensor_handle)
            vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(
                client_id, vision_sensor_handle)

            if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
                print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')

                # Get the transformed vision sensor image captured in correct format
                try:
                    transformed_image = task_1b.transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    transformed_depth_image = transform_vision_sensor_depth_image(
                        vision_sensor_depth_image, depth_image_resolution)

                    if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):

                        berries_dictionary = detect_berries(
                            transformed_image, transformed_depth_image)
                        print("Berries Dictionary = ", berries_dictionary)
                        berry_positions_dictionary = detect_berry_positions(
                            berries_dictionary)
                        print("Berry Positions Dictionary = ",
                              berry_positions_dictionary)

                        labelled_image = get_labeled_image(
                            transformed_image, berries_dictionary, berry_positions_dictionary)

                        cv2.imshow('transformed image', transformed_image)
                        cv2.imshow('transformed depth image',
                                   transformed_depth_image)
                        cv2.imshow('labelled image', labelled_image)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                    else:
                        print(
                            '\n[ERROR] transform_vision_sensor_image function is not configured correctly, check the code.')
                        print('Stop the CoppeliaSim simulation manually.')
                        print()
                        sys.exit()

                except Exception:
                    print(
                        '\n[ERROR] Your transform_vision_sensor_image function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print(
                    '\n[ERROR] get_vision_sensor function is not configured correctly, check the code.')
                print('Stop the CoppeliaSim simulation manually.')
                print()
                sys.exit()

        except Exception:
            print(
                '\n[ERROR] Your get_vision_sensor_image function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    cv2.destroyAllWindows()

    # Ending the Simulation
    try:
        return_code = task_1b.stop_simulation(client_id)

        if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
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
            print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
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
