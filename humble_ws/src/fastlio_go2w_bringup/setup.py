from glob import glob

from setuptools import find_packages, setup

package_name = "fastlio_go2w_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Koki Tanaka",
    maintainer_email="67k.tanaka@gmail.com",
    description="FAST-LIO + Livox MID-360 bringup for GO2-W.",
    license="MIT",
    tests_require=["pytest", "numpy"],
    entry_points={
        "console_scripts": [
            "fastlio_odom_adapter = fastlio_go2w_bringup.odom_adapter:main",
        ],
    },
)
