import os
from glob import glob
from setuptools import setup

package_name = "vehicle_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="person_follow_ws",
    maintainer_email="maintainer@example.com",
    description="ROS 2 driver for the YK CAN115 chassis",
    license="Apache-2.0",
    entry_points={"console_scripts": ["chassis_node = vehicle_bringup.chassis_node:main"]},
)

