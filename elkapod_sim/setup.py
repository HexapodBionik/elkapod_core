from setuptools import setup

package_name = 'elkapod_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/flat_world.wbt']))
data_files.append(('share/' + package_name + '/protos', ['protos/ElkapodSimplified.proto']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + "/" + package_name, ['elkapod_sim/elkapod_comm_server.py']))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piotr Patek',
    maintainer_email='hexapod.bionik@gmail.com',
    description='Elkapod simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elkapod_comm_server = elkapod_sim.webots_bringup:main'
        ],
    },
)
