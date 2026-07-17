from glob import glob
import os

from setuptools import setup

package_name = 'krac_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kch',
    maintainer_email='kch@todo.todo',
    description='Package for drone vision using YOLOv8',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'survivor_yolo = krac_vision.survivor_yolo:main',
            'landing_marker = krac_vision.landing_marker:main',
            'yolo_node = krac_vision.yolo_detector:main',
            'vision_tracker = krac_vision.vision_tracker:main',
        ],
    },
)
