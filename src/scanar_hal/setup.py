from setuptools import setup
import os

package_name = 'scanar_hal'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='callen0817',
    maintainer_email='callen0817@gmail.com',
    description='ScanAR Hardware Abstraction Layer Interfaces and Mock Implementations',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
