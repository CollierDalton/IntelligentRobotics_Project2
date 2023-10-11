# Import necessary libraries
import rospy
import sys
import select
import termios
import tty
import math
import random

# Import specific message types from ROS packages
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Define instructions for using the robot
instructions = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

# Define key bindings for movement (linear and angular velocities)
moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

# Define key bindings for speed adjustments
speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

# Function to get keyboard input
def get_key_press():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
    return key

# Global variable declarations for robot control
linear_speed = 0.2
angular_speed = 0.3
bot_center = 2
is_moving_forward = False
is_turning = False
is_avoiding_obstacle = False
is_busy = False
distance_moved = 0

# Define variables for tracking robot position and orientation
current_x_position = 0
current_y_position = 0
current_z_position = 0
previous_x_position = 0
previous_y_position = 0
previous_z_position = 0
current_angular_direction = 0
previous_angular_direction = 0
target_angular_difference = 0

# Define variables for minimum scan ranges from laser data
min_scan_ranges_three_sections = [8, 8, 8]
min_scan_ranges_two_sections = [8, 8]

#ANYTIME YOU SEE * 3.28, IT IS CONVERTING METERS TO FEET (1 FOOT = ABOUT 3.28 METERS)
# Distance formula for 3D, input in meters but output in feet
def calculate_distance(x2, y2, z2, x1, y1, z1):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) * 3.28

#1st Priority - Halt if collision detected by bumper
# Callback function for bumper sensor data
def handle_bumper_sensor(msg):
    if msg.bumper > 0:
        rospy.signal_shutdown("Collision")
        print("COLLISION!!! Robot has detected a collision and has stopped.")

# Declare global variables to store current position and angular direction.
    global current_x_position, current_y_position, current_z_position, current_angular_direction
    
    # Extract the pose data from the 'msg' object.
    pose = msg.pose.pose
    
    # Extract the position and orientation data from the pose.
    position = pose.position
    orientation = pose.orientation
    
    # Update the current x, y, and z positions based on the received data.
    current_x_position = position.x
    current_y_position = position.y
    current_z_position = position.z
    
    # Extract the 'w' (real) and 'z' (imaginary) components of the quaternion orientation.
    w = orientation.w
    z = orientation.z
    
    # Calculate the current angular direction (in degrees) using the arctan2 function.
    # The result is converted from radians to degrees using a conversion factor.
    current_angular_direction = 360 / math.pi * math.atan2(z, w)

# Callback function for laser data
def handle_laser_scan(msg):
    global min_scan_ranges_three_sections, min_scan_ranges_two_sections

    # Calculate the number of ranges in three sections and two sections
    num_ranges_three_sections = len(msg.ranges) // 3
    num_ranges_two_sections = len(msg.ranges) // 2

    # Divide the laser ranges into three sections and two sections
    divided_ranges_three_sections = divide_ranges(msg.ranges, num_ranges_three_sections)
    divided_ranges_two_sections = divide_ranges(msg.ranges, num_ranges_two_sections)

    # Filter out NaN (Not-a-Number) values from the divided ranges
    filtered_ranges_three_sections = filter_ranges(divided_ranges_three_sections)
    filtered_ranges_two_sections = filter_ranges(divided_ranges_two_sections)

    # Calculate the minimum ranges for three sections and two sections,
    # converting the result from meters to feet (multiply by 3.28)
    min_scan_ranges_three_sections = calculate_min_ranges(filtered_ranges_three_sections)
    min_scan_ranges_two_sections = calculate_min_ranges(filtered_ranges_two_sections)

# Helper function to divide laser scan ranges into sections
def divide_ranges(ranges, num_sections):
    return [ranges[i:i + num_sections] for i in range(0, len(ranges), num_sections)]

# Helper function to filter out NaN values from laser scan data
def filter_ranges(ranges_list):
    return [[x for x in subarray if not math.isnan(x)] for subarray in ranges_list]

# Helper function to calculate minimum ranges from filtered laser scan data
def calculate_min_ranges(filtered_ranges):
    return [min(x) * 3.28 if x else 8 for x in filtered_ranges]

# Main program entry point
if __name__ == "__main__":
    # Store terminal settings to restore later
    terminal_settings = termios.tcgetattr(sys.stdin)

    # Initialize ROS node and set up publishers/subscribers
    rospy.init_node('turtlebot_teleop')
    publisher = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
    bumper_sensor_subscriber = rospy.Subscriber('/mobile_base/sensors/core', SensorState, handle_bumper_sensor)
    odometer_subscriber = rospy.Subscriber('/odom', Odometry, handle_odometer_data)
    laser_scan_subscriber = rospy.Subscriber('/scan', LaserScan, handle_laser_scan)

    # Initialize control variables
    linear_velocity = 0
    angular_velocity = 0
    angular_velocity_constant = 0
    target_linear_speed = 0
    target_angular_speed = 0
    control_linear_speed = 0
    control_angular_speed = 0

    try:
        # Display instructions
        print(instructions)

        # Main loop for robot control
        while True:
			# Get the key pressed from a function (not shown here)
		    key_press = get_key_press()
		    
			#2nd Priority - Accept keyboard movements from user input
		    # Check if the Ctrl+C key combination was pressed to break out of the loop
		    if key_press == '\x03':
		        break
		    
		    # Check if the key pressed corresponds to a movement command in the 'moveBindings' dictionary
		    elif key_press in moveBindings.keys():
		        # Reset movement and distance information
		        is_moving_forward = False
		        is_turning = False
		        distance_moved = 0
		        
		        # Update linear and angular velocities based on the key pressed
		        linear_velocity = moveBindings[key_press][0]
		        angular_velocity = moveBindings[key_press][1]
		    
		    # Check if the key pressed is either space (' ') or 'k'
		    elif key_press == ' ' or key_press == 'k':
		        # Reset movement and distance information
		        is_moving_forward = False
		        is_turning = False
		        distance_moved = 0
		        
		        # Set linear and angular velocities to zero, effectively stopping movement
		        linear_velocity = 0
		        angular_velocity = 0
		        
		        # Additionally, reset control speed values if applicable
		        control_linear_speed = 0
		        control_angular_speed = 0
			
			#3rd Priority - Avoid symmetric objects
			# Check if the robot is currently in the process of avoiding an obstacle and meets certain conditions
			elif is_avoiding_obstacle and ((min_scan_ranges_three_sections[1] > bot_center) or is_busy):
			    
			    # Check if the robot is not already busy (busy flag is used to prevent multiple actions)
			    if not is_busy:
			        is_busy = True
			        previous_angular_direction = current_angular_direction
			        
			        # Determine the direction to turn based on the minimum scan ranges
			        if min_scan_ranges_two_sections[1] < min_scan_ranges_two_sections[0]:
			            angular_velocity_constant = -1
			        else:
			            angular_velocity_constant = 1
			    
			    # Reset movement and distance information
			    is_moving_forward = False
			    is_turning = False
			    distance_moved = 0
			    
			    # Set a target angular difference for the robot's movement; avoid *symmetric* obstacles by turning 150 degrees
			    target_angular_difference = 150
			    
			    # Calculate the current angular difference between the robot's orientation and the previous direction
			    current_angular_difference = abs(current_angular_direction - previous_angular_direction)
			    
			    # Check if the current angular difference is less than the target
			    if current_angular_difference < target_angular_difference:
			        linear_velocity = 0
			        angular_velocity = angular_velocity_constant
			    else:
			        # If the target angular difference is reached, reset obstacle avoidance flags and velocities
			        is_avoiding_obstacle = False
			        is_busy = False
			        target_angular_difference = 0
			        linear_velocity = 0
			        angular_velocity = 0
			        angular_velocity_constant = 0
					
			#4th Priority - Avoid asymmetric objects (and just objects in general)
			# Check if any of the values in min_scan_ranges_three_sections are less than bot_center
			elif any(a < bot_center for a in min_scan_ranges_three_sections):
			
			    # Reset movement and distance information
			    is_moving_forward = False
			    is_turning = False
			    distance_moved = 0
			
			    # Find the minimum range value in min_scan_ranges_three_sections
			    min_range_value = min(b for b in min_scan_ranges_three_sections)
			
			    # Check conditions based on the minimum range value and sensor data
			    if min_scan_ranges_three_sections[1] == min_range_value or max(c for c in min_scan_ranges_three_sections) < bot_center:
			        # If the center range is the minimum or the maximum range is less than bot_center
			        # Set linear velocity to move backward and stop angular rotation
			        linear_velocity = -1.5
			        angular_velocity = 0
			        is_avoiding_obstacle = True
			    elif min_scan_ranges_three_sections[2] == min_range_value:
			        # If the rightmost range is the minimum
			        # Stop linear movement and rotate left
			        linear_velocity = 0
			        angular_velocity = -1
			    else:
			        # If the leftmost range is the minimum
			        # Stop linear movement and rotate right
			        linear_velocity = 0
			        angular_velocity = 1

			#5th Priority - Turn randomly 15 degrees every 1ft moved
			# Check if the robot has moved a distance of 1 unit or more
			elif distance_moved >= 1:
			    
			    # Check if the robot is not currently in a turning state
			    if not is_turning:
			        # Store the current angular direction as the previous direction
			        previous_angular_direction = current_angular_direction
			        
			        # Generate a random target angular difference between -15 and 15 degrees
			        target_angular_difference = random.randint(-15, 15)
			        
			        # Set flags to indicate that the robot is now turning and not moving forward
			        is_turning = True
			        is_moving_forward = False
			    
			    # Calculate the current angular difference between the current and previous angular directions
			    current_angular_difference = current_angular_direction - previous_angular_direction
			    
			    # Check if the target angular difference is positive
			    if target_angular_difference >= 0:
			        # If current angular difference is greater than or equal to the target
			        if current_angular_difference >= target_angular_difference:
			            # Turn is complete, reset turning flags and velocities
			            is_turning = False
			            distance_moved = 0
			            linear_velocity = 0
			            angular_velocity = 0
			        else:
			            # Continue turning right
			            linear_velocity = 0
			            angular_velocity = 1
			    else:
			        # If target angular difference is negative
			        # If current angular difference is less than or equal to the target
			        if current_angular_difference <= target_angular_difference:
			            # Turn is complete, reset turning flags and velocities
			            is_turning = False
			            distance_moved = 0
			            linear_velocity = 0
			            angular_velocity = 0
			        else:
			            # Continue turning left
			            linear_velocity = 0
			            angular_velocity = -1

			#6th Priority - Drive forward
			# If none of the previous conditions are met (i.e., the robot is neither avoiding obstacles nor turning)
			else:
			    # Check if the robot is not currently in a forward movement state
			    if not is_moving_forward:
			        # Store the current x, y, and z positions as previous positions
			        previous_x_position = current_x_position
			        previous_y_position = current_y_position
			        previous_z_position = current_z_position
			        
			        # Set a flag to indicate that the robot is now moving forward
			        is_moving_forward = True
			    
			    # Calculate the distance moved based on the change in x, y, and z positions
			    distance_moved = calculate_distance(current_x_position, current_y_position, current_z_position,
			                                        previous_x_position, previous_y_position, previous_z_position)
			    
			    # Set linear velocity to 1 (indicating forward movement) and angular velocity to 0 (no rotation)
			    linear_velocity = 1
			    angular_velocity = 0
			
			# Calculate target linear and angular speeds by multiplying them with the velocity values
			target_linear_speed = linear_speed * linear_velocity
			target_angular_speed = angular_speed * angular_velocity
			
			# Adjust the control linear speed to match the target linear speed
			if target_linear_speed > control_linear_speed:
			    # If the target linear speed is greater, increase the control linear speed gradually
			    control_linear_speed = min(target_linear_speed, control_linear_speed + 0.02)
			elif target_linear_speed < control_linear_speed:
			    # If the target linear speed is smaller, decrease the control linear speed gradually
			    control_linear_speed = max(target_linear_speed, control_linear_speed - 0.02)
			else:
			    # If the target linear speed is already reached, keep it unchanged
			    control_linear_speed = target_linear_speed
			
			# Adjust the control angular speed to match the target angular speed
			if target_angular_speed > control_angular_speed:
			    # If the target angular speed is greater, increase the control angular speed gradually
			    control_angular_speed = min(target_angular_speed, control_angular_speed + 0.1)
			elif target_angular_speed < control_angular_speed:
			    # If the target angular speed is smaller, decrease the control angular speed gradually
			    control_angular_speed = max(target_angular_speed, control_angular_speed - 0.1)
			else:
			    # If the target angular speed is already reached, keep it unchanged
			    control_angular_speed = target_angular_speed

			twist = Twist()
            twist.linear.x = control_linear_speed
            twist.linear.y = 0 
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_angular_speed
            publisher.publish(twist)

	   	    #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        publisher.publish(twist)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)

		            
