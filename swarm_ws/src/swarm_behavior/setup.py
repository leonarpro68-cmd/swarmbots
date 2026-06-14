import os
from glob import glob

from setuptools import setup

package_name = "swarm_behavior"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="leo",
    maintainer_email="leonarpro68@gmail.com",
    description="Go-to-goal reactivo con evasion LiDAR y modo lider-seguidor para Summit XLS.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "go_to_goal = swarm_behavior.go_to_goal:main",
        ],
    },
)
