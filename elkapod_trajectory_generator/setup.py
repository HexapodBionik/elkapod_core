from setuptools import find_packages, setup

package_name = 'elkapod_trajectory_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='antoni.przybylik@wp.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "elkapod_trajectory_generator = elkapod_trajectory_generator.elkapod_trajectory_generator:main",
        ],
    },
)
