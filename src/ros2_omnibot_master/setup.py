from setuptools import setup

package_name = 'ros2_omnibot_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='widayaka',
    maintainer_email='diptyawidayaka@gmail.com',
    description='omnibot - publisher node camera',
    license='MIT',
    entry_points={
        'console_scripts':[
            'omnibot_master_node = ros2_omnibot_master.omnibot_master:main',
        ],
    },
)