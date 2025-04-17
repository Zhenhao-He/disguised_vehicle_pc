from setuptools import find_packages, setup
import os
from glob import glob
import sys

package_name = 'image_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/launch.py')),
        (os.path.join('share', package_name, 'parking_test_images'), glob('parking_test_images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parking',
    maintainer_email='parking@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = image_publisher.image_publisher:main'
        ],
    },
)