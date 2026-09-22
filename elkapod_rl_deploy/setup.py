import os
from glob import glob

from setuptools import setup

package_name = "elkapod_rl_deploy"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join(package_name, "launch", "*.launch.py"))),
        (os.path.join("share", package_name, "config"),
         glob(os.path.join(package_name, "config", "*.yaml"))),
    ],
    install_requires=["setuptools", "torch", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="Piotr Patek",
    maintainer_email="piotrpatek17@gmail.com",
    description="Deployment of SKRL-trained RL polcies to control the Elkapod via ROS2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "policy_node = elkapod_rl_deploy.policy_node:main",
        ],
    },
)
