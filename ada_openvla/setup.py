# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from glob import glob
import os
from setuptools import find_packages, setup

package_name = "ada_openvla"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    # Include all launch files.
    (
        os.path.join("share", package_name, "launch"),
        glob(os.path.join("launch", "*launch.[pxy][yma]*")),
    ),
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="OpenVLA OFT (Online Fine-Tuning) package for ADA robot",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "openvla_oft_online = ada_openvla.openvla_oft_online:main",
        ],
    },
)
