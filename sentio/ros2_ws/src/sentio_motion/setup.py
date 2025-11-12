from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sentio_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'schema'), glob('schema/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'pyyaml'],
    zip_safe=True,
    maintainer='DevSora Deep-Tech Research',
    maintainer_email='dev@devsora.tech',
    description='Motion and actuation layer for SENTIO',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_bridge_node = sentio_motion.servo_bridge:main',
        ],
    },
)
