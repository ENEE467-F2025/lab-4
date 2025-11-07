import sys, select

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import transforms3d as t3d
from numpy.typing import NDArray

from simple_planner.ur3e_robotiq import UR3eRobotiq
# from ur3e_robotiq import UR3eRobotiq

from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)



# position and velocity limits for ur3e joints
UR3E_JOINT_LIMITS = {
    "shoulder_pan_joint": {
        "position": [-2*np.pi, 2*np.pi],
        "velocity": np.pi, # rad/s
    },
    "shoulder_lift_joint": {
        "position": [-2*np.pi, 2*np.pi],
        "velocity": np.pi, # rad/s
    },
    "elbow_joint": {
        "position": [-np.pi, np.pi],
        "velocity": np.pi, # rad/s
    },
    "wrist_1_joint": {
        "position": [-2*np.pi, 2*np.pi],
        "velocity": 2*np.pi, # rad/s
    },
    "wrist_2_joint": {
        "position": [-2*np.pi, 2*np.pi],
        "velocity": 2*np.pi, # rad/s
    },
    "wrist_3_joint": {
        "position": [-np.inf, np.inf],
        "velocity": 2*np.pi, # rad/s
    },
}

class SimplePlanner(Node):
    """A simple planner node.

    ...
    """

    def __init__(self):
        """Initialize the simple planner node.

        """

        super().__init__("simple_planner")

        # goal pose
        self.goal_pose: PoseStamped = None

        # class variable for storing current joint states
        self.ur3e_joint_states: dict = {}

        # planner parameters
        self.declare_parameter("ik_tolerance", 0.01)  # meters
        self.declare_parameter("max_iterations", 10000)
        self.declare_parameter("tolerance", 1e-2)
        self.declare_parameter("err_weights", [1.0, 1.0])
        self.ik_tolerance = self.get_parameter("ik_tolerance").get_parameter_value().double_value
        self.max_iterations = self.get_parameter("max_iterations").get_parameter_value().integer_value
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        self.err_weights = self.get_parameter("err_weights").get_parameter_value().double_array_value

        # create a joint state subscriber
        self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            callback=self.joint_state_callback,
            qos_profile=10
        )

        # create a joint trajectory publisher
        self.joint_trajectory_pub = self.create_publisher(
            msg_type=JointTrajectory,
            topic="/joint_trajectory_controller/joint_trajectory",
            qos_profile=10
        )

        self.create_timer(
            timer_period_sec=0.2, # 5 Hz
            callback=self.run
        )

        self.model = rtb.models.UR3() # previously UR3eRobotiq(); changed to use default model for UR3; 
                                      # something may be wrong with UR3eRobotiq IK,
                                      # as it returns IK solutions that yield very large pose errors

        # create interactive marker for IK
        self.marker_server = InteractiveMarkerServer(
            node=self,
            namespace="ee_pose"
        )
        int_marker = make_6_dof_marker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "ee_pose"
        int_marker.description = "Goal Pose for EE"

        # create an arrow
        marker = Marker()
        marker.type = Marker.ARROW
        marker.pose.position.x=-.25
        marker.scale.x = .5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.
        marker.color.g = 1.
        marker.color.b = 0.
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(control)

        self.marker_server.insert(
            marker=int_marker,
            feedback_callback=self.marker_callback,
            feedback_type=5
        )
        self.marker_server.applyChanges()
        
        # end __init__
    
    def run(self):
        """Main ``loop''

        """

        # read input (non-blocking)
        inpt=select.select([sys.stdin], [], [], 1)[0]

        if inpt:
            # flush input
            sys.stdin.readline()
        else:
            return
        
        if self.goal_pose is None:
            self.get_logger().info("No Goal Pose Recieved")
            return
        
        if self.ur3e_joint_states is None:
            self.get_logger().info("No Joint State Information Recieved")
            return
                
        # TODO (Ex. 2): compute IK to get joint states
        # TODO: get translation component of goal_pose class variable
        T1 = sm.SE3()

        # TODO: get orientation component of goal_pose class variable
        # sm.UnitQuaternion takes two arguments:
        # a scalar part (s) and a three-dimensional array (v)
        # set each to the respective object from goal_pose

        T2 = sm.UnitQuaternion().SE3() # cast to SE3

        # TODO: Compose translational and rotational components to get final
        # SE3
        T = sm.SE3() # 

        self.get_logger().info(f"Computing IK for Goal Pose:\n{T.rtvec()}")

        q_current = [self.ur3e_joint_states[joint] for joint in self.ur3e_joint_states.keys()]

        try:
            success = False
            q_guess = self.last_solution if hasattr(self, "last_solution") else q_current

            for i in range(self.max_iterations):
                sol = self.model.ik_LM(
                    T,
                    q0=q_guess,
                    start="base_link",
                    end="tool0"
                )[0]

                # handle non-finite IK solutions
                if not np.all(np.isfinite(sol)):
                    q_guess = q_current
                    continue

                # compute pose error; use SE3 error to account for both position and orientation
                # we plan to the end effector frame "tool0"
                fk_T = self.model.fkine(sol, end="tool0")
                if not (np.all(np.isfinite(fk_T.t)) and np.all(np.isfinite(fk_T.R))):
                    q_guess = q_current
                    continue
                err_vec, goal_t_err, goal_rot_err = compute_se3_error(T, fk_T, w_rot=self.err_weights[1], w_trans=self.err_weights[0])
                if not (np.all(np.isfinite(err_vec)) and np.all(np.isfinite(goal_t_err)) and np.all(np.isfinite(goal_rot_err))):
                    q_guess = q_current
                    continue
                goal_pose_err= float(np.linalg.norm(err_vec))
                self.get_logger().info(f"Iteration {i+1}: Goal Pose Error = {goal_pose_err:.6f} m")
                if goal_pose_err < self.ik_tolerance:
                    success = True
                    self.last_solution = sol
                    break
                else:
                    # update initial guess and retry
                    q_guess = sol

            if not success:
                self.get_logger().warn(f"IK did not converge within {self.max_iterations} iterations (final error: {goal_pose_err:.6f} m)")
                return

            self.get_logger().info(f"IK converged with error {goal_pose_err:.6f} m")

        except Exception as e:
            self.get_logger().error(f"IK Failed: {e}")
            return

        # only plan if IK succeeded within tolerance
        plan_msg = self.simple_planner(
            desired_js={
                "shoulder_pan_joint": sol[0],
                "shoulder_lift_joint": sol[1],
                "elbow_joint": sol[2],
                "wrist_1_joint": sol[3],
                "wrist_2_joint": sol[4],
                "wrist_3_joint": sol[5]
            },
            N=50,
            tau=5.0
        )


        self.get_logger().info("Publishing Computed Trajectory")
        self.joint_trajectory_pub.publish(plan_msg)

    def marker_callback(self, feedback: InteractiveMarkerFeedback):
        """Update goal pose whenever the interactive marker is moved."""
        if feedback.pose is not None:
            self.get_logger().info("Goal Pose Received, press Enter to execute")
            feedback_msg = PoseStamped()
            feedback_msg.pose = feedback.pose
            feedback_msg.header = feedback.header
            self.goal_pose = feedback_msg

    def joint_state_callback(self, msg: JointState):
        """Process joint states.

        """
        
        # JointState msg has the following fields:
        #
        # string[] name
        # float64[] position
        # float64[] velocity
        # float64[] effort
        #

        for joint_name in UR3E_JOINT_LIMITS.keys():
            # get index of joint
            idx = msg.name.index(joint_name)

            # update dictionary
            self.ur3e_joint_states[joint_name] = msg.position[idx]
    
    def simple_planner(
        self,
        desired_js: dict[float], # of the form {"joint_name": value,}
        N: int=20,
        tau: float=10.
    ) -> JointTrajectory:
        """A simple planner that plans lines in joint space.

        """
        
        traj = JointTrajectory()
        traj.points = [JointTrajectoryPoint() for _ in range(N)]
        traj.joint_names = list(self.ur3e_joint_states.keys())

        dt = tau/N

        # TODO (Ex. 2): create time stamps using np.linspace()
        stamps = np.array([]) # MODIFY

        # TODO (Ex. 2): fill the traj.points field for the N trajectory points
        # using stamps from above
        for i in range(N):
            traj.points[i].time_from_start.sec = 0.0 # Equation (6)
            nsec = 0.0 # ns; Equation (8)
            traj.points[i].time_from_start.nanosec = nsec

        # DO NOT MODIFY ANY CODE BLOCKS BELOW
        traj.points[i].positions = []
        for joint in self.ur3e_joint_states.keys():
            curr_js = self.ur3e_joint_states[joint]
            des_js = desired_js[joint]
            dh = np.linspace(curr_js, des_js, N)

            for i in range(N):
                traj.points[i].positions.append(dh[i])


        return traj

def pose_stamped_to_se3(pose_stamped_obj: PoseStamped) -> NDArray:
    """
    Helper function to convert a PoseStamped message to a 4x4 SE3 transformation matrix.
    """
    trans = [pose_stamped_obj.pose.position.x, pose_stamped_obj.pose.position.y, pose_stamped_obj.pose.position.z]
    quat = np.array([pose_stamped_obj.pose.orientation.w,
                     pose_stamped_obj.pose.orientation.x,
                     pose_stamped_obj.pose.orientation.y,
                     pose_stamped_obj.pose.orientation.z], dtype=float)
    if np.linalg.norm(quat) == 0:
        quat = np.array([1., 0., 0., 0.])
    else:
        quat = quat / np.linalg.norm(quat)

    R_mat = t3d.quaternions.quat2mat(quat)
    # re-orthonormalize rotation to avoid numerical drift
    try:
        U, _, Vt = np.linalg.svd(R_mat)
        R_orth = U @ Vt
        if np.linalg.det(R_orth) < 0:
            U[:, -1] *= -1
            R_orth = U @ Vt
        R_mat = R_orth
    except Exception:
        pass
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = trans
    return T  

def compute_se3_error(T_desired, T_actual, w_trans=1.0, w_rot=1.0):
    """Compute combined SE3 error as weighted translation + rotation magnitude
    """
    R_d, t_d = T_desired.R, T_desired.t
    R_a, t_a = T_actual.R, T_actual.t

    # translation
    e_t = (t_a - t_d).flatten()

    # rotation error using matrix log map
    R_err = R_d.T @ R_a
    e_R = sm.SO3(R_err).log()  

    # weighted combined error
    print(f"Translation Error: {e_t}, Rotation Error: {e_R}")
    err = np.sqrt(w_trans * np.dot(e_t, e_t) + w_rot * np.dot(e_R, e_R))
    return err, e_t, e_R

def make_6_dof_marker():
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'base_link'
    int_marker.scale = .5

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.707
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    control.name = 'rotate_x'
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.707
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    control.name = 'move_x'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.0
    control.orientation.y = 0.707
    control.orientation.z = 0.0
    control.name = 'rotate_z'
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.0
    control.orientation.y = 0.707
    control.orientation.z = 0.0
    control.name = 'move_z'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 0.707
    control.name = 'rotate_y'
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 0.707
    control.name = 'move_y'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    return int_marker


def main():
    rclpy.init()
    node = SimplePlanner()
    rclpy.spin(node)


if __name__=="__main__":
    main()
    
#