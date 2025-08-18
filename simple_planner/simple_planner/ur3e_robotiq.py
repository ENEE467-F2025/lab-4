import os

import numpy as np
from roboticstoolbox.robot.ERobot import ERobot

from ament_index_python.packages import get_package_share_directory


class UR3eRobotiq(ERobot):

    def __init__(self):

        pkg_dir = get_package_share_directory("ur3e_robotiq_description")
        urdf_path = os.path.join(pkg_dir, "urdf", "ur3e_robotiq.urdf")

        args = super().URDF_read(urdf_path)

        super().__init__(
            args[0],
            name=args[1],
            urdf_string=args[2],
            urdf_filepath=args[3],
            gripper_links=[args[0][3], args[0][4]],
        )

        self.addconfiguration(
            "qr", np.array([0., -np.pi/2, 0., -np.pi/2, 0., 0.])
        )
        self.addconfiguration(
            "qz", np.zeros(shape=(6,), dtype=float)
        )


if __name__ == "__main__":

    robot = UR3eRobotiq()
    print(robot)

    qr = np.array([0., -np.pi/2, 0., -np.pi/2, 0., 0.])

    T = robot.fkine(
        qr,
        start="base_link",
        end="robotiq_hande_end"
    )
    print(T)

    sol=robot.ik_LM(
        T,
        start="base_link",
        end="robotiq_hande_end"
    )
    print(sol[0])

    print(type(T))