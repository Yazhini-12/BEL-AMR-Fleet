from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'bel_amr_fleet'


def collect_model_files(model_name):
    data_files = []

    model_root = os.path.join('models', model_name)

    for root, dirs, files in os.walk(model_root):
        if not files:
            continue

        install_dir = os.path.join(
            'share',
            package_name,
            root
        )

        source_files = [
            os.path.join(root, filename)
            for filename in files
        ]

        data_files.append(
            (install_dir, source_files)
        )

    return data_files


data_files = [
    # ROS package index
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
]


# Install every file inside all robot models
for model_name in [
    'tugbot',
    'amr_01',
    'amr_02',
    'amr_03'
]:
    data_files.extend(
        collect_model_files(model_name)
    )


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=data_files,

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
        'robot_controller = bel_amr_fleet.robot_controller:main',
	'priority_manager = bel_amr_fleet.priority_manager:main',
    ],
},
    
)
