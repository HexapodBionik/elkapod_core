from setuptools import find_packages, setup

package_name = "elkapod_comm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Piotr Patek",
    maintainer_email="hexapod.bionik@gmail.com",
    description="Provides communication services with Hardware Controller for rest of the Elkapod System",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "elkapod_comm_server = elkapod_comm.elkapod_comm_server:main"
        ],
    },
)
