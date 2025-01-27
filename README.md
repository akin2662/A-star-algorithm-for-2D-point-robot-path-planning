# A-star-algorithm-for-2D-point-robot-path-planning
**Note:** This project was a requirement for the course ENPM 661- Planning for Autonomous Robots at University of Maryland, College Park and was done in collaboration with Raajith Gadam (raajithg@umd.edu)

## Project Description:
This script contains an implementation of the A-star algorithm for a point robot for navigation around obstacles in a 2D space.

## Dependencies:

* python 3.11 (any version above 3 should work)
* Python running IDE (We used Pycharm)

## Libraries used:
* NumPy
* Time
* HeapQ
* OpenCv

## Instructions 
1. Download the zip file and extract it
	
2. Install python and the required dependencies: 

	`pip install numpy opencv-python`
	
3. Run the code or use desired python IDE:

	`$python3 a_star_raajithg_advaitk.py`

Input the clearance, radius, the coordinates and orientation of the robot at start and goal positions and the step size for the robot when  prompted in the terminal.
The instructions on how to input the coordinates and the orientation will be given in the terminal itself
**NOTE:** When you input the coordinates and the orientation for the start and the goal position, input them such that each value if followed by the next by after a single space. For example if the start position is (10,10,30) where x = 10, y = 10 , angle = 30, then enter it as **10 10 30**. Follow the same procedure for the goal node

**NOTE:** The step size of the robot should be between 1 and 10

**General Note**: The code best runs when the clearance and the robot radius is 1, otherwise, it takes some time, please be patient :)


