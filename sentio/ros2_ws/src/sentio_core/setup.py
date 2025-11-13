from setuptools import setup

package_name = 'sentio_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='DevSora',
    author_email='dev@devsora.tech',
    maintainer='DevSora',
    maintainer_email='dev@devsora.tech',
    description='SENTIO Core Package',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={},
)
