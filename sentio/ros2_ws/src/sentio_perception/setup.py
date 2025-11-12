from setuptools import setup
import os
from glob import glob

package_name = 'sentio_perception'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DevSora Deep-Tech Research',
    maintainer_email='dev@devsora.tech',
    description='Multimodal perception for SENTIO emotion detection',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_emotion_node = sentio_perception.face_emotion_node:main',
            'audio_emotion_node = sentio_perception.audio_emotion_node:main',
            'group_detector_node = sentio_perception.group_detector_node:main',
        ],
    },
)
