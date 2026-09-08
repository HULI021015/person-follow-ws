from setuptools import find_packages, setup

package_name = "yk_can_sdk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="person_follow_ws",
    maintainer_email="maintainer@example.com",
    description="USR-CAN115 four-wheel chassis SDK",
    license="Apache-2.0",
)

