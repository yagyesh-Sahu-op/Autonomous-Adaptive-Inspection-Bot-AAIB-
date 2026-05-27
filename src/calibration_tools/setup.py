from setuptools import setup
import os
from glob import glob

package_name = 'calibration_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot User',
    maintainer_email='user@example.com',
    description='Odometry calibration tools for differential drive robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_calibration_node = calibration_tools.odometry_calibration_node:main',
            'calibration_client = calibration_tools.calibration_client:main',
        ],
    },
)
