from setuptools import setup

package_name = 'exp'
submodules = "exp/talk_modules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mao-y',
    maintainer_email='cgjg0289@mail4.doshisha.ac.jp',
    description='exp',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "exp = exp.exp:main"
        ],
    },
)
