from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'elkapod_bringup'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))

setup(
    name=package_name,
    version='1.8.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piotr Patek',
    maintainer_email='piotrpatek17@gmail.com',
    description='Bringup package for Elkapod walking robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "elkapod_bringup = elkapod_bringup.__init__.py"
        ],
    },
)
