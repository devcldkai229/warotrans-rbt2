import os

from setuptools import find_packages, setup

package_name = "warotrans_rbt002_control_gateway"


def web_files():
    """Install the built UI (repo web/dist, copied to ./web by tools/build_web.sh) if present."""
    out = []
    root = "web"
    for dirpath, _, files in os.walk(root):
        if files:
            out.append((os.path.join("share", package_name, dirpath), [os.path.join(dirpath, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ] + web_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WaroTrans Team",
    maintainer_email="dev@warotrans.local",
    description="RBT-002 mobile control gateway",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_gateway = warotrans_rbt002_control_gateway.main:main",
        ],
    },
)
