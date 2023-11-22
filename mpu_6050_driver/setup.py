#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

#d = generate_distutils_setup(
#    packages=['mpu_6050_driver'],
#    scripts=['scripts/imu_node.py'],
#    package_dir={'': 'src'},
#    install_requires=['rospkg'],
#    )

#setup(**d)

package_name = 'mpu_6050_driver'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Name',
    maintainer_email='your@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
