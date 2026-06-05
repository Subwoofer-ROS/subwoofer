from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'subwoofer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*"))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sondre Flakstad',
    maintainer_email='SondreFlakstad@outlook.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "subwoofer = subwoofer.subwoofer:main",
            "trot_controller = subwoofer.trot_controller:main",
            "stance_controller = subwoofer.stance_controller:main",
            "state_manager = subwoofer.state_manager:main",
            "remote_controller = subwoofer.remote_controller:main"
        ],
    },
)
