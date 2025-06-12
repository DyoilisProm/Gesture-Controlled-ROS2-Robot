from setuptools import find_packages, setup

package_name = 'odometer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_file.py']),
        ('share/' + package_name + '/config', ['config/odometry_view.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tasodio4',
    maintainer_email='tasodio4@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometer = odometer.odometer_node:main',
            'square = odometer.motion_planner_node:main',
        ],
    },
)
