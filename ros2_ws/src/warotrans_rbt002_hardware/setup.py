from setuptools import find_packages, setup

package_name = "warotrans_rbt002_hardware"

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
    maintainer="WaroTrans Team",
    maintainer_email="dev@warotrans.local",
    description="RBT-002 base controller, kinematics, sensors and odometry",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_controller_node = warotrans_rbt002_hardware.base_controller_node:main",
            "sensor_adapter_node = warotrans_rbt002_hardware.sensor_adapter_node:main",
            "odom_node = warotrans_rbt002_hardware.odom_node:main",
        ],
    },
)
