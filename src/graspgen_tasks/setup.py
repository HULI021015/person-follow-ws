from glob import glob
from setuptools import find_packages, setup

package_name = "graspgen_tasks"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zheng",
    maintainer_email="zheng@example.com",
    description="Grasp task state machine",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "grasp_task_node = graspgen_tasks.grasp_task_node:main",
            "coordinate_debug_node = graspgen_tasks.coordinate_debug_node:main",
        ],
    },
)
