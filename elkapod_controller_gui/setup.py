from setuptools import find_packages, setup

package_name = "elkapod_controller_gui"

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
    maintainer="Antoni Przybylik",
    maintainer_email="antoni.przybylik@wp.pl",
    description="Simple Elkapod controller GUI",
    license="GPL",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "elkapod_controller_gui = elkapod_controller_gui.elkapod_controller_gui:main"
        ],
    },
)
