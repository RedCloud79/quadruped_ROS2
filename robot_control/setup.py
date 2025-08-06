from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='120488365+RedCloud79@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_point_server = robot_control.move_point_server:main',
            'home_server = robot_control.home_server:main',
            'docking_server = robot_control.docking_server:main',
            'patrol_server = robot_control.patrol_server:main',
            'emergency_server = robot_control.emergency_server:main',
            'stop_server = robot_control.stop_server:main',
        ],
    },
)
