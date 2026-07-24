from setuptools import setup
import os
from glob import glob

package_name = 'elp_camera_driver'

setup(
    name=package_name,
    version='1.5.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scanar',
    maintainer_email='scanar@viture.com',
    description='ELP 5MP Global Shutter Camera ROS 2 Driver for ScanAR C',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elp_camera_node = src.elp_camera_node:main',
        ],
    },
)
