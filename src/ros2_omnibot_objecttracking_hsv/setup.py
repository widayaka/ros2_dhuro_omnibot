from setuptools import setup

package_name = 'ros2_omnibot_objecttracking_hsv'

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
    entry_points={
        'console_scripts':[
            'omnibot_objecttracking_hsv_node = ros2_omnibot_objecttracking_hsv.omnibot_objecttracking_hsv:main',
        ],
    },
)