from setuptools import find_packages, setup

package_name = 'ur5_control'

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
    maintainer='hsinlin1',
    maintainer_email='hslin0307@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'move_to_position = ur5_control.move_to_position:main',
                'get_position = ur5_control.get_position:main',
        ],
    },
)
