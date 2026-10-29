import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pai_final_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='norihiro',
    maintainer_email='norihiro.vsq@gmail.com',
    description=(
        'Vision-guided TurtleBot3 scavenger hunt using pretrained '
        'YOLO-World open-vocabulary object detection.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
