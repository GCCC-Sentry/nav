from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_through_poses_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='3499874911@qq.com',
    description='Nav2 NavigateThroughPoses client for point selection',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_through_poses_node = nav_through_poses_client.nav_through_poses_node:main',
        ],
    },
)
