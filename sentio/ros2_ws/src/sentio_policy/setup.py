from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sentio_policy'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DevSora Deep-Tech Research',
    maintainer_email='dev@devsora.tech',
    description='Policy engine and behavior mapping for SENTIO',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_engine_node = sentio_policy.policy_engine:main',
        ],
    },
)
