from setuptools import find_packages, setup

package_name = 'lingbot_engine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scanarstereo',
    maintainer_email='callen0817@gmail.com',
    description='ScanAR G LingBot Tracking Engine Node',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lingbot_engine_node = lingbot_engine.lingbot_engine_node:main',
        ],
    },
)
