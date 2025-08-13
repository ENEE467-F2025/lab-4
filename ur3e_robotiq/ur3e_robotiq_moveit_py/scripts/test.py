#!/usr/bin/env python3

import rclpy
from rclpy.logging import get_logger

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters
)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.kinematic_constraints import construct_joint_constraint

from geometry_msgs.msg import Pose

import time

import numpy as np

pi=3.14159265359

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

def main():

    # moveit_config = (
    #     MoveItConfigsBuilder("ur3e_robotiq", package_name="ur3e_robotiq_moveit_config")
    #     .moveit_cpp(file_path="/home/ryan/Research/sim_ws/src/dexterous-manipulation/ur3e_robotiq/ur3e_robotiq_moveit_py/config/moveit_cpp.yaml")
    # ).to_dict()
    # moveit_config["use_sim_time"] = True
    
    rclpy.init()

    ur3e_robotiq = MoveItPy(node_name="moveit_py") #, config_dict=moveit_config)
    ur3e = ur3e_robotiq.get_planning_component("ur3e")
    logger = get_logger("moveit_py.pose_goal")

    robot_model = ur3e_robotiq.get_robot_model()
    robot_state = RobotState(robot_model)

    # pose_goal = Pose()
    # pose_goal.position.x = 0.1
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.5
    # pose_goal.orientation.w = 1.0

    # robot_state.set_from_ik("ur3e", pose_goal, "wrist_3_link")
    # robot_state.update()

    joint_vals = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -pi/2,
        "elbow_joint": 0.0,
        "wrist_1_joint": -pi/2,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    }

    for key in robot_state.joint_positions.keys():
        logger.info(f"{key}: {robot_state.joint_positions[key]}")

    
    robot_state.joint_positions=joint_vals

    ur3e.set_goal_state(robot_state=robot_state)

    plan_and_execute(ur3e_robotiq, ur3e, logger, sleep_time=3.0)
    
    while 1:

        ur3e.set_start_state_to_current_state()

        for key in joint_vals:
            joint_vals[key] += np.random.normal()

        robot_state.joint_positions=joint_vals
        ur3e.set_goal_state(robot_state=robot_state)

        plan_and_execute(ur3e_robotiq, ur3e, logger, sleep_time=1.0)


if __name__ == "__main__":
    main()