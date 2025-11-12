from setuptools import setup
import os
from glob import glob

package_name = 'sentio_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'udev_rules'), glob('udev_rules/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DevSora Deep-Tech Research',
    maintainer_email='dev@devsora.tech',
    description='Low-level hardware drivers for SENTIO sensors',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tfmini_node = sentio_drivers.tfmini_node:main',
            'maxsonar_node = sentio_drivers.maxsonar_node:main',
            'imu_node = sentio_drivers.imu_node:main',
            'uvc_camera_node = sentio_drivers.uvc_camera_node:main',
        ],
    },
)
