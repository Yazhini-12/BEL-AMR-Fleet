from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'bel_amr_fleet'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        # ROS 2 package index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            os.path.join('share', package_name),
            ['package.xml']
        ),

        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        # World files
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),

        # TugBot model files ONLY
        # Do not use glob('models/tugbot/*') because
        # it also matches the meshes directory.
        (
            os.path.join(
                'share',
                package_name,
                'models',
                'tugbot'
            ),
            [
                'models/tugbot/model.config',
                'models/tugbot/model.sdf',
            ]
        ),

        # TugBot base meshes
        (
            os.path.join(
                'share',
                package_name,
                'models',
                'tugbot',
                'meshes',
                'base'
            ),
            glob('models/tugbot/meshes/base/*')
        ),

        # TugBot light meshes
        (
            os.path.join(
                'share',
                package_name,
                'models',
                'tugbot',
                'meshes',
                'light_link'
            ),
            glob('models/tugbot/meshes/light_link/*')
        ),

        # TugBot gripper meshes
        (
            os.path.join(
                'share',
                package_name,
                'models',
                'tugbot',
                'meshes',
                'gripper2'
            ),
            glob('models/tugbot/meshes/gripper2/*')
        ),

        # TugBot wheel meshes
        (
            os.path.join(
                'share',
                package_name,
                'models',
                'tugbot',
                'meshes',
                'wheel'
            ),
            glob('models/tugbot/meshes/wheel/*')
        ),

        # TugBot VLP16 meshes
        (
            os.path.join(
                'share',
                package_name,
                'models',
                'tugbot',
                'meshes'
            ),
            glob('models/tugbot/meshes/*.dae')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='yazhini',
    maintainer_email='yazhini@example.com',

    description='Decentralized AMR Fleet Coordination System',

    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'robot_status = bel_amr_fleet.robot_status:main',
            'peer_communication = bel_amr_fleet.peer_communication:main',
        ],
    },
)