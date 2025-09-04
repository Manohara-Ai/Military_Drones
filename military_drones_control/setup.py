from setuptools import find_packages, setup

package_name = 'military_drones_control'

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
    maintainer='manohara',
    maintainer_email='manohara01012005@gmail.com',
    description='Nodes to control the military drones',
    license='MIT License',
    extras_require={"test": ["pytest"]},
    entry_points={
        'console_scripts': [
            'flight_controller = military_drones_control.flight_controller:main',
            'central_planner = military_drones_control.central_planner:main',
            'object_recognizer = military_drones_control.object_recognizer:main',
            'drone_gui = military_drones_control.drone_gui:main'
        ],
    },
)
