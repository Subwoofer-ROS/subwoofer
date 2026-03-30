from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'subwoofer_cv'

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
        (
            os.path.join("share", package_name, "statics"),
            glob(os.path.join("statics", "*"))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sondre Meiland-Flakstad',
    maintainer_email='SondreFlakstad@outlook.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "FaceDetection = subwoofer.face_detection:main",
            "BackgroundSubtraction = subwoofer.image_motion:main",
            "EdgeDetection = subwoofer.edge_detection:main",
        ],
    },
)
