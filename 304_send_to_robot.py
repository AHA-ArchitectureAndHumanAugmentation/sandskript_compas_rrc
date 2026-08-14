from pathlib import Path

import compas_rrc as rrc
from compas.data import json_load
from compas.geometry import Frame, Vector, Sphere, Polyline, Point, Line
from compas_viewer import Viewer
from compas.colors import Color

############## Constants ##############

HOME_CONFIG = [-89.68, -8.48, -191.38, -0.58, 22.8, -0.25]
TOOL_NAME = "t_SprayingTool"
WORK_OBJECT = "wobj0"

SPRAY_OUTPUT = "ABB_Scalable_IO_0_DO1"
PUMP_OUTPUT = "ABB_Scalable_IO_0_DO2"

HOME_SPEED = 100
APPROACH_SPEED = 100
TOOLPATH_SPEED = 100

SAFE_OFFSET = 100.0
TOOLPATH_OFFSET = Vector(0,0,0)

# 0 = Communication only
# 1 = Read current robot position
# 2 = Move to HOME_CONFIG
# 3 = Move to safe frame
# 4 = Move to first toolpath frame
# 5 = Follow complete toolpath
TEST_STEP = 5

VISUALIZE_TOOLPATH = False
RUN_ROBOT_AFTER_VIEWER = True
 
############## Functions ##############

def visualize_toolpath(frames, sphere_radius=936.888, frame_axis_length=80.0, point_radius=8.0, show_every_nth_frame=1):
    if not frames:
        raise ValueError("Cannot visualize an empty toolpath.")

    sphere = Sphere(sphere_radius)
    toolpath_polyline = Polyline([frame.point for frame in frames])
    viewer = Viewer()
    viewer.unit = "mm"

    ############## Sphere ##############
    viewer.scene.add(sphere, facecolor=Color(0.82, 0.82, 0.82), opacity=0.20)

    ############## Toolpath Polyline ##############
    viewer.scene.add(toolpath_polyline, linecolor=Color.blue(), linewidth=4)

    ############## Toolpath Points and Directions ##############
    for index, frame in enumerate(frames):
        # A small sphere showing the toolpath point
        point_marker = Sphere(point_radius, frame=Frame(frame.point, frame.xaxis, frame.yaxis))
        viewer.scene.add(point_marker, facecolor=Color.white())

        # Show only every Nth frame to avoid too much clutter
        if index % show_every_nth_frame != 0:
            continue

        point = frame.point

        x_end = point + frame.xaxis * frame_axis_length
        y_end = point + frame.yaxis * frame_axis_length
        z_end = point + frame.zaxis * frame_axis_length

        x_axis_line = Line(point, x_end)
        y_axis_line = Line(point, y_end)
        z_axis_line = Line(point, z_end)

        # X direction
        viewer.scene.add(x_axis_line, linecolor=Color.red(), linewidth=4)
        # Y direction
        viewer.scene.add(y_axis_line, linecolor=Color.green(), linewidth=4)
        # Z direction — normally the most important tool direction
        viewer.scene.add(z_axis_line, linecolor=Color.blue(), linewidth=6)

    ############## Safe Frames ##############

    first_safe_frame = get_safe_frame(frames[0])
    final_safe_frame = get_safe_frame(frames[-1])

    viewer.scene.add(first_safe_frame)
    viewer.scene.add(final_safe_frame)

    print("\nOpening COMPAS Viewer...")
    print("Sphere radius:", sphere_radius, "mm")
    print("Number of toolpath frames:", len(frames))
    print("Frame axis length:", frame_axis_length, "mm")
    print("Point marker radius:", point_radius, "mm")

    viewer.show()

def get_safe_frame(frame, offset=SAFE_OFFSET):
    safe_frame = Frame(frame.point, frame.xaxis, frame.yaxis)
    return safe_frame.translated(-frame.zaxis * offset)

def load_toolpath(filename):
    data = json_load(filename)

    if "frames" not in data:
        raise KeyError("The JSON file does not contain 'frames'.")

    frame_data = data["frames"]
    frames = []

    for item in frame_data:
        if isinstance(item, Frame):
            # Original format: a direct list of Frame objects
            frames.append(item)
        elif isinstance(item, dict) and "frame" in item:
            # Numbered format:
            # {"number": 1, "frame": Frame(...)}
            frames.append(item["frame"])
        else:
            raise TypeError(
                "Invalid toolpath item. Expected a Frame or a dictionary "
                "containing a 'frame' key, but received: "
                f"{type(item).__name__}"
            )
    return frames

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

def apply_fixed_orientation(frames, orientation_frame):
    fixed_frames = []
    for frame in frames:
        fixed_frame = Frame(point=frame.point, xaxis=orientation_frame.xaxis, yaxis=orientation_frame.yaxis)
        fixed_frames.append(fixed_frame)
    return fixed_frames

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

    ############## Use JSON Frames Directly ##############

    first_frame = frames[0]

    # Keep the orientation of the accepted first toolpath frame
    # for the complete toolpath.
    frames = apply_fixed_orientation(frames, first_frame)

    first_frame = frames[0]
    safe_frame = get_safe_frame(first_frame)

    print("\nUsing JSON frames without modification.")
    print("First JSON frame:", first_frame)
    print("First JSON point:", first_frame.point)
    print("First JSON X-axis:", first_frame.xaxis)
    print("First JSON Y-axis:", first_frame.yaxis)
    print("First JSON Z-axis:", first_frame.zaxis)

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

    # without air pressure on ##
    for index, frame in enumerate(remaining_frames, start=2):
        is_last_frame = index == len(frames)

        print("Sending frame", index, "of", len(frames), flush=True,)

        if is_last_frame:
            abb.send_and_wait(rrc.MoveToFrame(frame, speed=TOOLPATH_SPEED, zone=rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR))
        else:
            abb.send(rrc.MoveToFrame(frame, speed=TOOLPATH_SPEED, zone=rrc.Zone.Z10, motion_type=rrc.Motion.LINEAR))
    # without air pressure on ##

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
data_file = Path(__file__).resolve().parent / "data" / "toolpath_2.1.json"
toolpath_frames = load_toolpath(data_file)
toolpath_frames = [frame.translated(TOOLPATH_OFFSET) for frame in toolpath_frames]
check_toolpath(toolpath_frames)

############## COMPAS Viewer ##############
if VISUALIZE_TOOLPATH:
    visualize_toolpath(toolpath_frames, sphere_radius=936.888, frame_axis_length=100.0, point_radius=10.0, show_every_nth_frame=1)
    if not RUN_ROBOT_AFTER_VIEWER:
        print("\nVisualization completed. Robot movement was not started.")
        raise SystemExit
 
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