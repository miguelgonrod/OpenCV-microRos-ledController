from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'opencv_led'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'modules'), glob('opencv_led/*Module.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel Gonzalez',
    maintainer_email='miguel_gonzalezr@ieee.org',
    description='This package is a publisher to a micro-ros node that receives 0 or 1 to blink a led, the status is given by the number of fingers',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fingerCounter = opencv_led.fingerCounter:main",
            "handTrackingModule = opencv_led.handTrackingModule:main"
        ],
    },
)
