from setuptools import find_packages, setup
import os
from glob import glob

package_name = 't12_prj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='alancheng1226164539',
    maintainer_email='hcheng57@asu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ttb_nav = t12_prj.ttb_nav:main',
            'block_detector = t12_prj.gui:main',  
            'cmd_controller = t12_prj.cmd_controller:main',
            'turtlebot_state = t12_prj.turtlebot_state:main'
        ],
    },
)
