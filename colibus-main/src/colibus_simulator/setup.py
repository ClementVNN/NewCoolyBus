from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'colibus_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_node=colibus_simulator.vehicle_node.py:main'
            'vlp16=colibus_simulator.vlp16.py:main'
            'camera_rear=colibus_simulator.camera_rear.py:main'
            'drive_node=colibus_simulator.drive_node.py:main'
            'video_feed=colibus_simulator.video_feed.py:main'
            'auto_drive=colibus_simulator.Auto_Drive.py:main'
            'ekf=colibus_simulator.ekf.py:main'
            'speed_accel=colibus_simulator.speed_accel.py:main'
            
        ],
    },
)
