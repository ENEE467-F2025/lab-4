from setuptools import find_packages, setup
import os

package_name = 'simple_spawner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/bridge.yaml']),
        (os.path.join('share', package_name, 'config'), ['config/bridge_spawner.yaml']),
        (os.path.join('share', package_name, 'rviz'), ['rviz/view_tf.rviz']),
        (os.path.join('share', package_name, 'rviz'), ['rviz/base.rviz']),
        (os.path.join('share', package_name, 'launch'), ['launch/spawner.launch.py']),
        (os.path.join('share', package_name, 'models', 'cuboid'), [
            'models/cuboid/model.config',
            'models/cuboid/model.sdf',
        ]),
        (os.path.join('share', package_name, 'models', 'cylinder'), [
            'models/cylinder/model.config',
            'models/cylinder/model.sdf',
        ]),
        (os.path.join('share', package_name, 'models', 'sphere'), [
            'models/sphere/model.config',
            'models/sphere/model.sdf',
        ]),
        (os.path.join('share', package_name, 'models', 'cube'), [
            'models/cube/model.config',
            'models/cube/model.sdf',
        ]),
        (os.path.join('share', package_name, 'worlds'), ['worlds/spawner_world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drce',
    maintainer_email='enwerem@terpmail.umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_spawner_node = simple_spawner.simple_spawner:main'
        ],
    },
)
