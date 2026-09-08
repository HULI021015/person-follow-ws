import os
from glob import glob
from setuptools import setup

package_name = "person_follower"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="person_follow_ws",
    maintainer_email="maintainer@example.com",
    description="Low-latency Dahua NetSDK YOLOv8/BoT-SORT person follower",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "person_follower_node = person_follower.node:main",
            "camera_latency_test = person_follower.camera_test:main",
        ]
    },
)
