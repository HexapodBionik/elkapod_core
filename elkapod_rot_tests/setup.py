from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elkapod_rot_tests'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name), glob(os.path.join(package_name, '*.py'))))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piotr Patek',
    maintainer_email='piotrpatek17@gmail.com',
    description='Gait generator prototype for Elkapod Walking Robot developed at WUT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elkapod_rot = elkapod_rot_tests.elkapod_rot:main',
        ],
    },
)
