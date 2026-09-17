from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'bel_amr_fleet'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

            data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),

        # =========================
        # AMR-01
        # =========================
        (
            os.path.join('share', package_name, 'models', 'amr_01'),
            glob('models/amr_01/*.sdf')
        ),

        (
            os.path.join(
                'share', package_name,
                'models', 'amr_01', 'meshes'
            ),
            glob('models/amr_01/meshes/*')
        ),

        # =========================
        # AMR-02
        # =========================
        (
            os.path.join('share', package_name, 'models', 'amr_02'),
            glob('models/amr_02/*.sdf')
        ),

        (
            os.path.join(
                'share', package_name,
                'models', 'amr_02', 'meshes'
            ),
            glob('models/amr_02/meshes/*')
        ),

        # =========================
        # AMR-03
        # =========================
        (
            os.path.join('share', package_name, 'models', 'amr_03'),
            glob('models/amr_03/*.sdf')
        ),

        (
            os.path.join(
                'share', package_name,
                'models', 'amr_03', 'meshes'
            ),
            glob('models/amr_03/meshes/*')
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
        'robot_controller = bel_amr_fleet.robot_controller:main',
    ],
},
    
)
