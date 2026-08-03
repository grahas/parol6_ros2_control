from setuptools import find_packages, setup

package_name = "parol6_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gupyfish",
    maintainer_email="gupyfish@gmail.com",
    description=(
        "Local TCP bridge between the parol6_hardware_interface ros2_control "
        "plugin and a parol6-server instance."
    ),
    license="MIT",
    # `tests_require` (the old mechanism) is silently dropped by modern
    # setuptools -- colcon's ament_python test task decides between its
    # pytest and legacy-unittest steps by checking `extras_require['test']`
    # (and `tests_require`, but that one never survives setuptools' own
    # metadata parsing anymore), so this is what actually makes
    # `colcon test` pick pytest instead of silently running 0 unittest
    # tests via `python setup.py test`.
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "parol6_bridge = parol6_bridge.bridge_node:main",
        ],
    },
)
