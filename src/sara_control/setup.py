from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sara_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sevin',
    maintainer_email='sevin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            
            'sara_viz_bridge = sara_control.sara_viz_bridge:main',
            'servo_controller = sara_control.servo_controller:main',
            'navigation = sara_control.navigation:main',
            'mission_start = sara_control.mission_start:main',
            'guidance = sara_control.guidance:main',
            'autopilot = sara_control.autopilot:main',
            'safety = sara_control.safety:main',
            'vehicle_sim = sara_control.vehicle_sim:main',
            'telemetry = sara_control.telemetry:main',
        


        ],
    },
)
