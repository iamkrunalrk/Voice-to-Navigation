from setuptools import setup
from glob import glob
import os

package_name = 'sjtu_drone_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "rviz"), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'webapp'), glob('webapp/*')),
        (os.path.join('share', package_name, "prompts"), glob('prompts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krunalrathod',
    maintainer_email='www.krunalrathod@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_drone = sjtu_drone_bringup.spawn_drone:main',
            'rosgpt = sjtu_drone_bringup.rosgpt:main',
            'rosgptparser_drone = sjtu_drone_bringup.rosgptparser_drone:main',
            'rosgpt_client_node = sjtu_drone_bringup.rosgpt_client_node:main',

        ],
    },
)
