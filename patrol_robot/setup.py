from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'patrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 安装目录下的所有文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nanimi',
    maintainer_email='nanimi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = patrol_robot.patrol_node:main',
            'audio_player_node = patrol_robot.audio_player_node:main',
        ],
    },
)
