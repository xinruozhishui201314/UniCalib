from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'unicalib'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'sensor_templates'),
            glob('config/sensor_templates/*.yaml')),
        (os.path.join('share', package_name, 'config', 'vehicle_configs'),
            glob('config/vehicle_configs/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UniCalib Team',
    maintainer_email='calib@example.com',
    description='UniCalib: Unified Multi-Sensor Automatic Calibration System',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unicalib_node = unicalib.ros2_node:main',
            'run_calibration = scripts.run_calibration:main',
        ],
    },
)
