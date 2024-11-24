from setuptools import find_packages, setup

package_name = "drone_motion_commander_pkg"

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
    maintainer="ubuntu",
    maintainer_email="patrik.persson@live.com",
    description="A motion commander for the drone",
    license="GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drone_motion_commander = drone_motion_commander_pkg.drone_motion_commander:main"
        ],
    },
)
