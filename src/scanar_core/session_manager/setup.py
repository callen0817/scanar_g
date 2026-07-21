from setuptools import find_packages, setup

package_name = 'session_manager'

setup(
    name=package_name,
    version='3.1.0',
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
    description='ScanAR Dual main session orchestrator node.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'session_manager_node = session_manager.session_manager_node:main'
        ],
    },
)
