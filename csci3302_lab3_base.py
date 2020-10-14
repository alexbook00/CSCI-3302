"""robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor
import csci3302_lab3_supervisor
import numpy as np

# Robot Pose Values
pose_x = 0
pose_y = 0
pose_theta = 0

# Constants to help with the Odometry update
WHEEL_FORWARD = 1
WHEEL_STOPPED = 0
WHEEL_BACKWARD = -1

# state = "turn_drive_turn_control" # Start in the easier, non-feedback control case!
state = "feedback_control" # End with the more complex feedback control method!
sub_state = "bearing" # TODO: It may be helpful to use sub_state to designate operation modes within the "turn_drive_turn_control" state

# create the Robot instance.
csci3302_lab3_supervisor.init_supervisor()
robot = csci3302_lab3_supervisor.supervisor

EPUCK_MAX_WHEEL_SPEED = 0.12880519 # Unnecessarily precise ePuck speed in m/s. REMINDER: motor.setVelocity() takes ROTATIONS/SEC as param, not m/s.
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_WHEEL_RADIUS = 0.0205 # ePuck's wheels are 0.041m in diameter.

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


def update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed):
    '''
    Given the amount of time passed and the direction each wheel was rotating,
    update the robot's pose information accordingly
    '''
    global pose_x, pose_y, pose_theta, EPUCK_MAX_WHEEL_SPEED, EPUCK_AXLE_DIAMETER
    pose_theta += (right_wheel_direction - left_wheel_direction) * time_elapsed * (EPUCK_MAX_WHEEL_SPEED) / EPUCK_AXLE_DIAMETER
    pose_x += math.cos(pose_theta) * time_elapsed * EPUCK_MAX_WHEEL_SPEED * (left_wheel_direction + right_wheel_direction)/2
    pose_y += math.sin(pose_theta) * time_elapsed * EPUCK_MAX_WHEEL_SPEED * (left_wheel_direction + right_wheel_direction)/2
    pose_theta = get_bounded_theta(pose_theta)

def get_bounded_theta(theta):
    '''
    Returns theta bounded in [-PI, PI]
    '''
    while theta > math.pi: theta -= 2.*math.pi
    while theta < -math.pi: theta += 2.*math.pi
    return theta

def main():
    global robot, state, sub_state
    global leftMotor, rightMotor, SIM_TIMESTEP, WHEEL_FORWARD, WHEEL_STOPPED, WHEEL_BACKWARD
    global pose_x, pose_y, pose_theta

    last_odometry_update_time = None

    # Keep track of which direction each wheel is turning
    left_wheel_direction = WHEEL_STOPPED
    right_wheel_direction = WHEEL_STOPPED

    # Important IK Variable storing final desired pose
    target_pose = None # Populated by the supervisor, only when the target is moved.


    # Sensor burn-in period
    for i in range(10): robot.step(SIM_TIMESTEP)

    # Main Control Loop:
    while robot.step(SIM_TIMESTEP) != -1:
        # Odometry update code -- do not modify
        if last_odometry_update_time is None:
            last_odometry_update_time = robot.getTime()
        time_elapsed = robot.getTime() - last_odometry_update_time
        update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed)
        last_odometry_update_time = robot.getTime()

        # Get target location -- do not modify
        if target_pose is None:
            target_pose = csci3302_lab3_supervisor.supervisor_get_relative_target_pose()
            print("New IK Goal Received! Target: %s" % str(target_pose))

        # Your code starts here
        x_g = target_pose[0]
        y_g = target_pose[1]
        theta_g = target_pose[2]

        bearing_error = get_bounded_theta(np.arctan2((y_g - pose_y), (x_g - pose_x)) - pose_theta)
        distance_error = np.sqrt((pose_x - x_g)**2 + (pose_y - y_g)**2)
        heading_error = get_bounded_theta(theta_g - pose_theta)

        if state == "turn_drive_turn_control":
            print("DISTANCE ERROR:", distance_error)
            if sub_state == 'bearing':
                if bearing_error < -.01:
                    leftMotor.setVelocity(leftMotor.getMaxVelocity()/10)
                    rightMotor.setVelocity(-rightMotor.getMaxVelocity()/10)
                    left_wheel_direction = WHEEL_FORWARD/10
                    right_wheel_direction = WHEEL_BACKWARD/10
                elif bearing_error > .01:
                    leftMotor.setVelocity(-leftMotor.getMaxVelocity()/10)
                    rightMotor.setVelocity(rightMotor.getMaxVelocity()/10)
                    left_wheel_direction = WHEEL_BACKWARD/10
                    right_wheel_direction = WHEEL_FORWARD/10
                else:
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    left_wheel_direction = WHEEL_STOPPED/10
                    right_wheel_direction = WHEEL_STOPPED/10
                    sub_state = 'distance'

            elif sub_state == 'distance':
                if distance_error > .01:
                    leftMotor.setVelocity(leftMotor.getMaxVelocity()/10)
                    rightMotor.setVelocity(rightMotor.getMaxVelocity()/10)
                    left_wheel_direction = WHEEL_FORWARD/10
                    right_wheel_direction = WHEEL_FORWARD/10
                elif distance_error < .01:
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    left_wheel_direction = WHEEL_STOPPED/10
                    right_wheel_direction = WHEEL_STOPPED/10
                    sub_state = 'heading'

            elif sub_state == 'heading':
                if heading_error < -.01:
                    leftMotor.setVelocity(leftMotor.getMaxVelocity()/10)
                    rightMotor.setVelocity(-rightMotor.getMaxVelocity()/10)
                    left_wheel_direction = WHEEL_FORWARD/10
                    right_wheel_direction = WHEEL_BACKWARD/10
                elif heading_error > .01:
                    leftMotor.setVelocity(-leftMotor.getMaxVelocity()/10)
                    rightMotor.setVelocity(rightMotor.getMaxVelocity()/10)
                    left_wheel_direction = WHEEL_BACKWARD/10
                    right_wheel_direction = WHEEL_FORWARD/10
                else:
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    left_wheel_direction = WHEEL_STOPPED/10
                    right_wheel_direction = WHEEL_STOPPED/10
                    sub_state = ''
                    state = 'feedback_control'

        elif state == "feedback_control":
            if distance_error > .015:
                print('DISTANCE ERROR:',distance_error)
                distance_constant = .2
                if distance_error > .04:
                    phi_l = (distance_error*distance_constant - (bearing_error*EPUCK_AXLE_DIAMETER)/2)/EPUCK_WHEEL_RADIUS
                    phi_r = (distance_error*distance_constant + (bearing_error*EPUCK_AXLE_DIAMETER)/2)/EPUCK_WHEEL_RADIUS
                elif distance_error <= .04:
                    phi_l = (distance_error - (heading_error*EPUCK_AXLE_DIAMETER)/2)/EPUCK_WHEEL_RADIUS
                    phi_r = (distance_error + (heading_error*EPUCK_AXLE_DIAMETER)/2)/EPUCK_WHEEL_RADIUS

                if phi_l > phi_r:
                    leftMotor.setVelocity((leftMotor.getMaxVelocity()/10) * (phi_l/phi_r))
                    rightMotor.setVelocity((rightMotor.getMaxVelocity()/10))
                    left_wheel_direction = WHEEL_FORWARD/10 * (phi_l/phi_r)
                    right_wheel_direction = WHEEL_FORWARD/10
                elif phi_l < phi_r:
                    leftMotor.setVelocity((leftMotor.getMaxVelocity()/10))
                    rightMotor.setVelocity((rightMotor.getMaxVelocity()/10) * (phi_r/phi_l))
                    left_wheel_direction = WHEEL_FORWARD/10
                    right_wheel_direction = WHEEL_FORWARD/10 * (phi_r/phi_l)
                else:
                    leftMotor.setVelocity(leftMotor.getMaxVelocity()/10)
                    rightMotor.setVelocity(rightMotor.getMaxVelocity()/10)
                    left_wheel_direction = WHEEL_FORWARD/10
                    right_wheel_direction = WHEEL_FORWARD/10

            else:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                left_wheel_direction = WHEEL_STOPPED
                right_wheel_direction = WHEEL_STOPPED
        else:
            pass

        # To help you debug! Feel free to comment out.
        print("Current pose: [%5f, %5f, %5f]\t\t Target pose: [%5f, %5f, %5f]\t\t Errors: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta, target_pose[0], target_pose[1], target_pose[2],bearing_error, distance_error, heading_error))


if __name__ == "__main__":
    main()
