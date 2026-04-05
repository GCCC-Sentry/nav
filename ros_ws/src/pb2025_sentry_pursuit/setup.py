from setuptools import setup
import os
from glob import glob

package_name = 'pb2025_sentry_pursuit'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PolarBear',
    maintainer_email='user@todo.todo',
    description='哨兵追击决策节点：对接SP视觉系统，实现自动追击、禁区规避、车辆过滤功能',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pursuit_node = pb2025_sentry_pursuit.pursuit_node:main',
        ],
    },
)
