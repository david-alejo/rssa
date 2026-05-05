from setuptools import find_packages, setup

package_name = 'collision_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name+'/maps', ['maps/house.yaml']),
        ('share/'+package_name+'/maps', ['maps/house.pgm']),
        ('share/'+package_name+'/cfg', ['cfg/practica_4.rviz']),
        ('share/'+package_name+'/launch', ['launch/practica_4.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Alejo',
    maintainer_email='dalejo@us.es',
    description='Basic collision avoidance package for the turtlebot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'emergency_stop = collision_avoidance.emergency_stop:main',
            'DWA = collision_avoidance.DWA:main',
        ],
    },
)
