import glob

from setuptools import find_packages, setup

package_name = "parol6_control_services"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gupyfish",
    maintainer_email="gupyfish@gmail.com",
    description=(
        "ROS 2 Trigger services (home, gripper open/close) for one-off "
        "imperative PAROL6 operations, connecting directly to parol6-server."
    ),
    license="MIT",
    # See parol6_bridge/setup.py's comment: extras_require['test'] (not
    # tests_require) is what makes `colcon test` actually run pytest here.
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "control_services_node = parol6_control_services.control_services_node:main",
            "call_method_node = parol6_control_services.call_method_node:main",
        ],
    },
)
