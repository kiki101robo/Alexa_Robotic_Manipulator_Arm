from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'arduinobot_python_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirti',
    maintainer_email='kiki1@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = arduinobot_python_examples.simple_publisher:main',
            'simple_subscriber = arduinobot_python_examples.simple_subscriber:main',
            'simple_service_client = arduinobot_python_examples.simple_service_client:main',            
            'simple_service_server = arduinobot_python_examples.simple_service_server:main',
            'simple_parameter = arduinobot_python_examples.simple_parameter:main',
            'simple_action_server = arduinobot_python_examples.simple_action_server:main',
            'simple_action_client = arduinobot_python_examples.simple_action_client:main',
            'simple_moveit_interface = arduinobot_python_examples.simple_moveit_interface:main'
        ],
    },
)
