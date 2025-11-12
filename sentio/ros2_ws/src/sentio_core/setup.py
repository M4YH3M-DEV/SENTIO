from setuptools import setup
import os
from glob import glob

package_name = 'sentio_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if any
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DevSora Deep-Tech Research',
    maintainer_email='dev@devsora.tech',
    description='SENTIO Core - System manager and heartbeat node',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sentio_core_node = sentio_core.main:main',
        ],
    },
)