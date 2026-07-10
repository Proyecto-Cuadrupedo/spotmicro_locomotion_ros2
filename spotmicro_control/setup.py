from glob import glob
import os

from setuptools import find_packages, setup

package_name = "spotmicro_locomotion_ros2"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src", exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config", "gait"), glob("config/gait/*.yaml")),
        (os.path.join("share", package_name, "config", "joints"), glob("config/joints/*.yaml")),
        (os.path.join("share", package_name, "config", "links"), glob("config/links/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chaski",
    maintainer_email="d.martinezr2@uniandes.edu.co",
    description="ROS 2 Jazzy CHAMP locomotion integration for the SpotMicro simulation.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spotmicro_champ_bridge = spotmicro_locomotion_ros2.spotmicro_champ_bridge:main",
        ],
    },
)