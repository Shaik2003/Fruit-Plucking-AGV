# Fruit-Plucking-AGV
This drone was developed while participating in E-yantra 2020, IIT Bombay.

## Construction:
This robot was developed in python and simulated on CoppeliaSim. It navigates through a greenhouse randomly generated pathway, identifies fruits, and plucks the specified number of fruits from the plants using an attached arm. It contains Image processing algorithms to recognise the type of fruit, and read QR code for navigation. Works on two PID controllers for its control system. 

## Working:
Its given a specific amount of each fruit as input, and the trees are randomly placed in the greenhouse. The robot turns around and uses a single camera to store the number of fruits on each tree within its vicinity to recognise and visit least number of trees with the most fruit. It then navigates to each of them using PIDs, and then plucks the fuits and stores in its compartementalised basket. At the end, it comes and seperately srops the fruits.


Video: https://youtu.be/WHvZ4S-07xU
