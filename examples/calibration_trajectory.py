import numpy as np
from argparse import ArgumentParser

from franky import Affine, JointWaypointMotion, JointWaypoint, Robot, CartesianWaypointMotion, CartesianWaypoint, \
    ReferenceType, RobotPose

import time

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--host", default="192.168.0.4", help="FCI IP of the robot")
    args = parser.parse_args()

    # Connect to the robot
    robot = Robot(args.host)
    robot.relative_dynamics_factor = 0.05
    robot.recover_from_errors()

    while True:
        time.sleep(200)

    # current_pose = robot.current_pose()

    # Reduce the acceleration and velocity dynamic
    # robot.relative_dynamics_factor = 0.2

    # # Define and move forwards
    # wp_motion = CartesianWaypointMotion([
    #     CartesianWaypoint(RobotPose(Affine(np.array([0.0, 0.0, -0.12])), -0.2), ReferenceType.Relative),
    #     CartesianWaypoint(RobotPose(Affine(np.array([0.08, 0.0, 0.0])), 0.0), ReferenceType.Relative),
    #     CartesianWaypoint(RobotPose(Affine(np.array([0.0, 0.1, 0.0])), 0.0), ReferenceType.Relative),
    # ])

    # # You can try to block the robot now.
    # robot.move(wp_motion)


# def calibration_traj(t, pos_scale=0.1, angle_scale=0.2, hand_camera=False):
#     x = -np.abs(np.sin(3 * t)) * pos_scale
#     y = -0.8 * np.sin(2 * t) * pos_scale
#     z = 0.5 * np.sin(4 * t) * pos_scale
#     a = -np.sin(4 * t) * angle_scale
#     b = np.sin(3 * t) * angle_scale
#     c = np.sin(2 * t) * angle_scale
#     if hand_camera:
#         value = np.array([z, y, -x, c / 1.5, b / 1.5, -a / 1.5])
#     else:
#         value = np.array([x, y, z, a, b, c])
#     return value

# def change_pose_frame(pose, frame, degrees=False):
#     R_frame = euler_to_rmat(frame[3:6], degrees=degrees)
#     R_pose = euler_to_rmat(pose[3:6], degrees=degrees)
#     t_frame, t_pose = frame[:3], pose[:3]
#     euler_new = rmat_to_euler(R_frame @ R_pose, degrees=degrees)
#     t_new = R_frame @ t_pose + t_frame
#     result = np.concatenate([t_new, euler_new])
#     return result

# calib_pose = calibration_traj(i * step_size, hand_camera=hand_camera)
# desired_pose = change_pose_frame(calib_pose, pose_origin)