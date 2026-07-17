from setuptools import setup
import os
from glob import glob

package_name = 'krac_gimbal'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 폴더의 모든 .launch.py 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config(프리셋 각도 파라미터) 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kch',
    maintainer_email='kch@todo.todo',
    description='SIYI A8 mini gimbal control node (SIYI SDK over Ethernet/UDP)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run krac_gimbal gimbal_node
            'gimbal_node = krac_gimbal.gimbal_node:main',
            # ros2 run krac_gimbal camera_node  (RTSP → /camera/image_raw)
            'camera_node = krac_gimbal.camera_node:main',
            # ros2 run krac_gimbal approach_node  (60°→인식→90°직하 접근)
            'approach_node = krac_gimbal.approach_node:main',
        ],
    },
)
