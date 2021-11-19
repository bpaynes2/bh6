import sys
import os
import time
import threading
import pyrealsense2 as rs

#from kbhit import KBHit
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

# Calling kortex_api module, base_pb2, to make sure the arm is in servoing mode  
def move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
       
    # Move arm to ready position
    print("Moving the arm to a safe position")

    # Setting action type to 
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None

    # walking through action list to search for action labeled "home", once found. The robot will follow under the predetermined home movement
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

# calls base_pd2 module from kortex api to define a command variable as a twist command alias
def move_forward(base, sX, eX):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    # divided set object coordinate by .004, to find scale
    # this is used to to define a linear forward movement 
    twist.linear_z = (sX - eX) / 4.975 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    
    # Let time for twist to be executed
    time.sleep(5)
    base.Stop()
    time.sleep(1)

    return True

#sweeps workspace from left to right, 
def getDistance(base):
    move_horizontal(base,0.014,-0.197)
    move_vertical(base,0.434,0.137)

    #start_time = time.time()
    #end_time = time.time()
    command = Base_pb2.TwistCommand()
    twist = command.twist
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    minDistance = 10
    yCoordinate = 0
    for x in range(0,60):
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        width = int(depth.get_width())
        height = int(depth.get_height())
        #if not depth: continue

        distance = depth.get_distance(int(width /2),int(height / 2))
        if(minDistance > distance and distance != 0):
            minDistance = distance
            yCoordinate = 0.321 * (x / 59) - 0.124
        twist.linear_x = 0.04
        base.SendTwistCommand(command)
        time.sleep(0.1)
        print(minDistance, yCoordinate)
    return True, minDistance, yCoordinate

def move_to_object(base, sX, camDis):

    eX = sX + camDis - 0.12
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    # divided set object coordinate by .004, to find scale
    # this is used to to define a linear forward movement 
    twist.linear_z = (sX - eX) / 4.975 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    
    # Let time for twist to be executed
    time.sleep(5)
    base.Stop()
    time.sleep(1)

    return True, eX

def move_vertical(base, sZ, eZ):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0
    
    # Moving the arm down vertically
    twist = command.twist
    twist.linear_y = (sZ - eZ ) / 4.95 * -1
    print(twist.linear_y)

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True

def move_horizontal(base, sY, eY):
    
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0
    
    # Moving the arm down vertically
    twist = command.twist
    twist.linear_x = ( sY - eY ) / 5.175 * -1

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True, eY

class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def ResetGrippers(self):

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Resetting gripper to standard position
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0
        finger.value = position
        self.base.SendGripperCommand(gripper_command)
        time.sleep(1)

    def SendGripperCommands(self):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.35
        finger.value = position
        self.base.SendGripperCommand(gripper_command)
        time.sleep(1)


def main():
    #endX = float(endX)
    #endY = float(endY)
    #endZ = float(endZ)
    #print(endX, endY, endZ)
    #homeX, homeY, homeZ = 0.575, 0.014, 0.434
    #startX, startY, startZ = [float(s) for s in input("Enter the coordiantes your object is currently located at (x, y, z): ").split()]
    endX, endY, endZ = [float(s) for s in input("Enter the coordinates you would like to drop your object off at (x, y, z): ").split()]

    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        min = 0
        y = 0
        pickUpY = 0
        pickUpX = 0
        # Moving to home position
        success = True
        success &= move_to_home_position(base)
        success &= move_to_home_position(base)

        success, min, y = getDistance(base)

        #Moving to object after detection
        success, pickUpY = move_horizontal(base, 0.18, y)
        success, pickUpX = move_to_object(base, 0.572, min)
        example = GripperCommandExample(router)
        example.SendGripperCommands()
        
        success &= move_vertical(base,-0.434, -0.137)
        success = move_horizontal(base, pickUpY, endY)
        success = move_forward(base, pickUpX, endX)
        success &= move_vertical(base,0.434, 0.137)
        example.ResetGrippers()
        success &= move_vertical(base,-0.434,-0.137)
        success &= move_to_home_position(base)

        #Move object to end position and reset gripper
        #success &= move_horizontal(base, )

        return 0 if success else 1
if __name__ == "__main__":
    exit(main())