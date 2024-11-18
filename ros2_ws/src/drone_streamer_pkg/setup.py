from setuptools import find_packages, setup

package_name = "drone_streamer_pkg"

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
    description="enables streaming and control of crazyflie with ai-deck extension",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["drone_streamer = drone_streamer_pkg.drone_streamer:main"],
    },
)