from glob import glob
import os

from setuptools import find_packages, setup

package_name = "obstacle_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cursor Agent",
    maintainer_email="cursoragent@cursor.com",
    description="ROS 2 Jazzy + Gazebo waypoint navigation simulation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_generator = obstacle_avoidance.waypoint_generator:main",
            "navigator = obstacle_avoidance.navigator:main",
            "fake_lidar = obstacle_avoidance.fake_lidar:main",
            "progress_monitor = obstacle_avoidance.progress_monitor:main",
        ],
    },
)
