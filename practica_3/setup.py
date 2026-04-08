from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='charizard',
    maintainer_email='churr.te@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtlebot_controller = turtlebot_controller.turtlebot_controller:main',
            'path_publisher = turtlebot_controller.path_publisher:main'
        ],
    },
)
