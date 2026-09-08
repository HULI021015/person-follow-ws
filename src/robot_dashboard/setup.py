import os
from glob import glob
from setuptools import setup

package_name = "robot_dashboard"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "web"),
            [path for path in glob("web/*") if os.path.isfile(path)],
        ),
        (os.path.join("share", package_name, "web", "vendor"), glob("web/vendor/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zheng",
    maintainer_email="zheng@example.com",
    description="Local ROS 2 dashboard for mobile manipulator control",
    license="Apache-2.0",
    entry_points={"console_scripts": ["dashboard_node = robot_dashboard.dashboard_node:main"]},
)
