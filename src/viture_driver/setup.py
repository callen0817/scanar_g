from setuptools import setup

package_name = 'viture_driver'

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
    description='ScanAR G VITURE Glass Driver Package',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viture_driver_node = viture_driver.viture_driver_node:main'
        ],
    },
)
