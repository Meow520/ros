import os
from glob import glob
from setuptools import find_packages, setup

package_name = "text_generate"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
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
            "talker = text_generate.text_generate:main",
            "asr_listener = text_generate.asr_memory:main",
            "obj_listener = text_generate.obj_memory:main",
            "per_listener = text_generate.per_memory:main",
            "role = text_generate.role_setting:main"
        ],
    },
)
