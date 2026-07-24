from setuptools import setup
import os
from glob import glob

package_name = 'elp_camera_driver'

setup(
    name=package_name,
    version='1.5.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), ['src/elp_camera_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scanar',
    maintainer_email='scanar@viture.com',
    description='ELP 5MP Global Shutter Camera ROS 2 Driver for ScanAR C',
    license='Proprietary',
)
