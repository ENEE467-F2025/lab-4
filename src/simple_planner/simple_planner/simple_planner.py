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
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# from simple_planner.ur3e_robotiq import UR3eRobotiq
from ur3e_robotiq import UR3eRobotiq

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
        self.goal_pose: Pose = None

        # class variable for storing current joint states
        self.ur3e_joint_states: dict = {}

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

        self.model = UR3eRobotiq()


        # create interavtive marker for IK
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
        
        # compute IK to get joint states
        T1 = sm.SE3.Trans(
            self.goal_pose.position.x,
            self.goal_pose.position.y,
            self.goal_pose.position.z
        )
        T2 = sm.UnitQuaternion(
            s=self.goal_pose.orientation.w,
            v=[
                self.goal_pose.orientation.x,
                self.goal_pose.orientation.y,
                self.goal_pose.orientation.z
            ]
        ).SE3()

        T = T1*T2

        try:
            sol=self.model.ik_LM(
                T,
                start="base_link",
                end="robotiq_hande_end"
            )[0]
        except Exception as e:
            self.get_logger().info("IK Failed: Invalid Pose")
            return
        
        print(sol)

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
            T=5.
        )

        self.get_logger().info("Publishing Computed Trajectory")
        self.joint_trajectory_pub.publish(plan_msg)
            

    
    def marker_callback(self, feedback: InteractiveMarkerFeedback):
        """Get goal pose when mouse up.

        """

        # test
        self.get_logger().info("Goal Pose Recieved, Press Enter to Execute")
        self.goal_pose = feedback.pose

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
        T: float=10.
    ) -> JointTrajectory:
        """A simple planner that plans lines in joint space.

        """
        
        traj = JointTrajectory()
        traj.points = [JointTrajectoryPoint() for _ in range(N)]
        traj.joint_names = list(self.ur3e_joint_states.keys())

        dt = T/N
        stamps = np.linspace(dt, T, N)

        # time stamps
        for i in range(N):
            traj.points[i].time_from_start.sec = int(stamps[i])
            nsec = int(stamps[i]%1*1e9)
            traj.points[i].time_from_start.nanosec = nsec

        for joint in self.ur3e_joint_states.keys():
            curr_js = self.ur3e_joint_states[joint]
            des_js = desired_js[joint]
            dh = np.linspace(curr_js, des_js, N)

            for i in range(N):
                traj.points[i].positions.append(dh[i])


        return traj

        

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