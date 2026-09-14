from setuptools import find_packages, setup

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
    ],
    install_requires=['setuptools'],
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
