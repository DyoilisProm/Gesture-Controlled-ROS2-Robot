from setuptools import find_packages, setup

package_name = 'gesture_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/gesture_classifier.pkl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tasodio4',
    maintainer_email='tasodio4@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_control = gesture_control.gesture_prediction:main',
        ],
    },
)
