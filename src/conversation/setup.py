import os
from glob import glob
from setuptools import setup

package_name = "conversation"
submodules = "conversation/talk_modules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mao-y",
    maintainer_email="cgjg0289@mail4.doshisha.ac.jp",
    description="generate sentences",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talk = conversation.talk:main",
            "talk2robot = conversation.talk2robot:main",
            "asr_listener = conversation.asr_memory:main",
        ],
    },
)
