from setuptools import find_packages, setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name+'/cfg', ['cfg/global_costmap_params.yaml']),
        ('share/'+package_name+'/cfg', ['cfg/practica_5.rviz']),
        ('share/'+package_name+'/launch', ['launch/practica_5.launch.xml']),
        ('share/'+package_name+'/launch', ['launch/planner.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rssa',
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
            'path_planner = path_planner.path_planner_node:main',
        ],
    },
    scripts=['path_planner/dijkstra.py'],
)
