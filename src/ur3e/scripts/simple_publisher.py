import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np

class SimplePublisher(Node):
    """
    """

    def __init__(self):
        """

        """

        super().__init__("simple_publisher")

        # readability
        pub_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self._pub = self.create_publisher(
            msg_type=JointTrajectory,
            topic="/joint_trajectory_controller/joint_trajectory",
            qos_profile=pub_qos_profile
        )

        sub_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        )
        self._joint_state_sub = self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            callback=self._joint_state_callback,
            qos_profile=sub_qos_profile
        )

        self._timer = self.create_timer(
            timer_period_sec=1.,
            callback=self._pub_joint_trajectory
        )

        self._joint_states: dict = {}
    
    def _joint_state_callback(self, msg: JointState):
        """

        """

        for i, name in enumerate(msg.name):
            self._joint_states[name] = msg.position[i]
    
    def _pub_joint_trajectory(self):
        """

        """

        names = list(self._joint_states.keys())
        curr_pos = list(self._joint_states.values())
        curr_pos = np.array(curr_pos)

        msg = JointTrajectory()
        #msg.header.stamp = self.get_clock().now().to_msg()
        #msg.header.frame_id = "world"
        msg.joint_names = names

        N=10
        T=1.
        dQ=np.random.normal(0., .25, size=(6,))

        traj = np.linspace(curr_pos, curr_pos+dQ, N)
        dt = np.linspace(T/N, T, N)

        pts=[]
        for i in range(N):
            pt = JointTrajectoryPoint()
            pt.positions=traj[i]
            sec, nsec = divmod(dt[i], 1)
            pt.time_from_start.sec = int(sec)
            pt.time_from_start.nanosec = int(nsec*1e9)
            
            pts.append(pt)
        
        msg.points = pts

        self._pub.publish(msg)




#######################################################################

def main(args=None):
    rclpy.init()
    pubber=SimplePublisher()

    rclpy.spin(pubber)

if __name__=="__main__":
    main()