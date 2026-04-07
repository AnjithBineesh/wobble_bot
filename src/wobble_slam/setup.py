from glob import glob
import os

from setuptools import setup


package_name = 'wobble_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        # xacro / urdf files
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*')),
        # SLAM config
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wobble',
    maintainer_email='wobble@todo.todo',
    description='Wobble robot simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
