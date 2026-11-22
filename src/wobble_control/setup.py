from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'wobble_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wobble',
    maintainer_email='wobble@todo.todo',
    description='Wobble robot motor control package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_node = wobble_control.motor_node:main',
            'test_motors = wobble_control.test_motors:main',
        ],
    },
)
