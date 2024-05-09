from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'elkapod_motion'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'MotionPlanning/kinematics/'), glob('MotionPlanning/kinematics/*.py')))
data_files.append((os.path.join('share', package_name, 'MotionPlanning'), glob('MotionPlanning/*.*')))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damian',
    maintainer_email='baraniak.damian@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "elkapod_kinematics = elkapod_motion.elkapod_kinematics:main",
            "elkapod_translation = elkapod_motion.elkapod_position_translation:main"
        ],
    },
)
