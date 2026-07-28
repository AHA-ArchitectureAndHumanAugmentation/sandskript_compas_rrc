from pathlib import Path

import compas_rrc as rrc
from compas.data import json_load
from compas.geometry import Frame, Vector

############## Constants ##############

HOME_CONFIG = [90.0, 15.0, -150.0, -5.0, -40.0, -215.0]
TOOL_NAME = "t_SprayingTool"
WORK_OBJECT = "wobj_SprayingNet"

SPRAY_OUTPUT = "ABB_Scalable_IO_0_DO1"
PUMP_OUTPUT = "ABB_Scalable_IO_0_DO2"

HOME_SPEED = 300
APPROACH_SPEED = 200
TOOLPATH_SPEED = 600

SAFE_OFFSET = 200.0
TOOLPATH_OFFSET = Vector(-400.0, 0.0, 0.0)

# 0 = Communication only
# 1 = Read current robot position
# 2 = Move to HOME_CONFIG
# 3 = Move to safe frame
# 4 = Move to first toolpath frame
# 5 = Follow complete toolpath
TEST_STEP = 5

############## Functions ##############

def get_safe_frame(frame, offset=SAFE_OFFSET):
    safe_frame = Frame(frame.point, frame.xaxis, frame.yaxis)
    return safe_frame.translated(Vector(0, 0, offset))

def load_toolpath(filename):
    data = json_load(filename)
    if "frames" not in data:
        raise KeyError("The JSON file does not contain 'frames'.")
    return data["frames"]

def check_toolpath(frames):
    if not frames:
        raise ValueError("The toolpath contains no frames.")
    print("Number of toolpath frames:", len(frames))
    print("First toolpath frame:", frames[0])
    print("Last toolpath frame:", frames[-1])

def move_to_frame(abb, frame, speed, description):
    print("\nMoving to", description)
    print(frame)
    abb.send_and_wait(rrc.MoveToFrame(frame, speed=speed, zone=rrc.Zone.FINE))
    print(description, "completed.")

def apply_fixed_orientation(frames, orientation_frame):
    fixed_frames = []

    for frame in frames:
        fixed_frame = Frame(point=frame.point, xaxis=orientation_frame.xaxis, yaxis=orientation_frame.yaxis)
        fixed_frames.append(fixed_frame)

    return fixed_frames

def pump_on(abb):
    print("\nTurning pump ON.", flush=True)
    abb.send_and_wait(rrc.SetDigital(PUMP_OUTPUT, 1))
    print("Pump is ON.", flush=True)

def pump_off(abb):
    print("\nTurning pump OFF.", flush=True)
    abb.send_and_wait(rrc.SetDigital(PUMP_OUTPUT, 0))
    print("Pump is OFF.", flush=True)

def spray_on(abb):
    print("\nTurning spray valve ON.", flush=True)
    abb.send_and_wait(rrc.SetDigital(SPRAY_OUTPUT, 1))
    print("Spray valve is ON.", flush=True)

def spray_off(abb):
    print("\nTurning spray valve OFF.", flush=True)
    abb.send_and_wait(rrc.SetDigital(SPRAY_OUTPUT, 0))
    print("Spray valve is OFF.", flush=True)

def run_movement_test(abb, frames, test_step):
    if test_step not in range(6):
        raise ValueError("TEST_STEP must be between 0 and 5.")

    print("\nSelected TEST_STEP:", test_step)

    ############## STEP 0: Communication ##############

    abb.send_and_wait(rrc.PrintText("Python connected to ABB"))
    print("STEP 0 completed: ABB communication works.")
    if test_step == 0:
        return

    ############## STEP 1: Read Robot Position ##############

    robot_joints, external_axes = abb.send_and_wait(rrc.GetJoints())
    print("\nCurrent robot joints:", robot_joints)
    print("Current external axes:", external_axes)
    print("STEP 1 completed: Robot position received.")
    if test_step == 1:
        return

    ############## Prepare Robot ##############

    abb.send_and_wait(rrc.SetTool(TOOL_NAME))
    abb.send_and_wait(rrc.SetWorkObject(WORK_OBJECT))
    abb.send_and_wait(rrc.SetAcceleration(20, 20))

    ############## STEP 2: Move to Home ##############

    abb.send_and_wait(rrc.PrintText("Moving to home configuration"))

    abb.send_and_wait(rrc.MoveToJoints(HOME_CONFIG, [], speed=HOME_SPEED, zone=rrc.Zone.FINE))

    print("STEP 2 completed: Robot moved to HOME_CONFIG.")

    ############## Capture Home Nozzle Orientation ##############

    home_frame = abb.send_and_wait(rrc.GetFrame())

    print("\nHome TCP frame:", home_frame)
    print("Home nozzle X-axis:", home_frame.xaxis)
    print("Home nozzle Y-axis:", home_frame.yaxis)
    print("Home nozzle Z-axis:", home_frame.zaxis)

    # Keep the toolpath positions, but replace all orientations
    # with the TCP orientation measured at HOME_CONFIG.
    frames = apply_fixed_orientation(frames, home_frame)

    first_frame = frames[0]
    safe_frame = get_safe_frame(first_frame)

    print("\nFirst frame with home orientation:", first_frame)

    if test_step == 2:
        return

    ############## STEP 3: Move to Safe Frame ##############

    abb.send_and_wait(rrc.PrintText("Moving to safe toolpath frame"))
    move_to_frame(abb, safe_frame, APPROACH_SPEED, "safe toolpath frame")

    print("STEP 3 completed: Robot moved to safe frame.")
    if test_step == 3:
        return

    ############## STEP 4: Move to First Toolpath Frame ##############

    abb.send_and_wait(rrc.PrintText("Moving to first toolpath frame"))
    move_to_frame(abb, first_frame, APPROACH_SPEED,"first toolpath frame",)

    print("STEP 4 completed: Robot moved to first toolpath frame.")
    if test_step == 4:
        return

    ############## STEP 5: Follow Complete Toolpath ##############

    abb.send_and_wait(rrc.PrintText("Following movement toolpath"))

    remaining_frames = frames[1:]


    # # without air pressure on ##
    # for index, frame in enumerate(remaining_frames, start=2):
    #     is_last_frame = index == len(frames)

    #     print("Sending frame", index, "of", len(frames), flush=True,)

    #     if is_last_frame:
    #         abb.send_and_wait(rrc.MoveToFrame(frame, speed=TOOLPATH_SPEED, zone=rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR))
    #     else:
    #         abb.send(rrc.MoveToFrame(frame, speed=TOOLPATH_SPEED, zone=rrc.Zone.Z10, motion_type=rrc.Motion.LINEAR))
    # # without air pressure on ##


    # ## with air pressure on ##
    # try:
    #     spray_on(abb)
    #     for index, frame in enumerate(remaining_frames, start=2):
    #         is_last_frame = index == len(frames)

    #         print("Sending frame", index, "of", len(frames), flush=True)

    #         if is_last_frame:
    #             # Wait until the robot reaches the final frame
    #             # before turning the spray off.
    #             abb.send_and_wait(
    #                 rrc.MoveToFrame(frame, speed=TOOLPATH_SPEED, zone=rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR))
    #         else:
    #             # Queue intermediate movements for continuous motion.
    #             abb.send(rrc.MoveToFrame(frame, speed=TOOLPATH_SPEED, zone=rrc.Zone.Z10, motion_type=rrc.Motion.LINEAR))
    # finally:
    #     # Turn the spray off even if the movement raises an error.
    #     spray_off(abb)
    # ## with air pressure on ##


    ## with pump & air pressure on ##
    pump_started = False
    spray_started = False

    PUMP_START_DELAY = 2.0

    try:
        # Start the pump first.
        pump_on(abb)
        pump_started = True

        # Wait for material to travel through the hose.
        print("\nWaiting 2 seconds before opening the air valve.", flush=True)
        abb.send_and_wait(rrc.WaitTime(PUMP_START_DELAY))

        # Open the air valve after the material arrives.
        spray_on(abb)
        spray_started = True

        for index, frame in enumerate(remaining_frames, start=2):
            is_last_frame = index == len(frames)

            print("Sending frame", index, "of", len(frames), flush=True)

            command = rrc.MoveToFrame(frame, speed=TOOLPATH_SPEED, zone=rrc.Zone.FINE if is_last_frame else rrc.Zone.Z10, motion_type=rrc.Motion.LINEAR)

            if is_last_frame:
                abb.send_and_wait(command)
            else:
                abb.send(command)

    finally:
        if pump_started:
            try:
                pump_off(abb)
            except Exception as error:
                print("Could not turn pump OFF:", error, flush=True)

        if spray_started:
            try:
                spray_off(abb)
            except Exception as error:
                print("Could not turn spray OFF:", error, flush=True)
    ## with pump & air pressure on ##



    joints, external_axes = abb.send_and_wait(rrc.GetJoints())

    print("Complete toolpath finished.", flush=True)
    print("Final toolpath robot joints:", joints, flush=True)

    ############## Return to Home ##############

    final_safe_frame = get_safe_frame(frames[-1])

    abb.send_and_wait(rrc.PrintText("Retracting from toolpath"))
    move_to_frame(abb, final_safe_frame, APPROACH_SPEED, "final safe frame")

    abb.send_and_wait(rrc.PrintText("Returning to home configuration"))

    abb.send_and_wait(rrc.MoveToJoints(HOME_CONFIG, [], speed=HOME_SPEED, zone=rrc.Zone.FINE))

    final_joints, final_external_axes = abb.send_and_wait(rrc.GetJoints())

    print("Robot returned to HOME_CONFIG.", flush=True)
    print("Final robot joints:", final_joints, flush=True)

############## Main Script ##############

data_file = Path(__file__).resolve().parent / "data" / "toolpath.json"
toolpath_frames = load_toolpath(data_file)
toolpath_frames = [frame.translated(TOOLPATH_OFFSET) for frame in toolpath_frames]
check_toolpath(toolpath_frames)

############## ABB Robot ##############

ros = None

try:
    print("\nConnecting to ROS...")
    ros = rrc.RosClient()
    ros.run()
    abb = rrc.AbbClient(ros, "/rob1")
    print("Connected to ROS.")
    print("Starting movement test.")
    run_movement_test(abb, toolpath_frames, TEST_STEP)
    print("\n=== SELECTED MOVEMENT TEST COMPLETED SUCCESSFULLY ===")

except KeyboardInterrupt:
    print("\nMovement test stopped manually.")

except Exception as error:
    print("\nROBOT MOVEMENT TEST FAILED:", type(error).__name__, error, flush=True)

finally:
    if ros is not None:
        try:
            if ros.is_connected:
                ros.close()
        except Exception as close_error:
            print("Could not close ROS:", close_error)