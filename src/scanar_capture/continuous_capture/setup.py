from setuptools import find_packages, setup

package_name = 'continuous_capture'

setup(
    name=package_name,
    version='3.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scanarstereo',
    maintainer_email='scanarstereo@todo.todo',
    description='ScanAR continuous trajectory capture and export package',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'continuous_capture_node = continuous_capture.continuous_capture_node:main'
        ],
    },
)
