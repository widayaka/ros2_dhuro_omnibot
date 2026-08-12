from setuptools import setup
from glob import glob
import os

package_name = 'ros2_omnibot_linefollowing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='widayaka',
    maintainer_email='diptyawidayaka@gmail.com',
    description='omnibot - line following node',
    license='MIT',

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],

    entry_points={
        'console_scripts':[
            'omnibot_linefollowing_node = ros2_omnibot_linefollowing.omnibot_linefollowing:main',
        ],
    },
)