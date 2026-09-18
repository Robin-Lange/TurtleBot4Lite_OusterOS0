from glob import glob
from setuptools import setup

package_name = "tb4_ouster_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RobinLange",
    maintainer_email="rl11732@georgiasouthern.edu",
    description="Stationary diagnostics, perception, and safety authority gate for Create 3 with Ouster OS0.",
    license="Unspecified",
    entry_points={
        "console_scripts": [
            "stationary_diagnostics = tb4_ouster_autonomy.stationary_diagnostics:main",
            "cloud_filter = tb4_ouster_autonomy.cloud_filter:main",
            "safety_authority_gate = tb4_ouster_autonomy.safety_authority_gate:main",
            "overnight_monitor = tb4_ouster_autonomy.overnight_monitor:main",
            "teleop_keyboard = tb4_ouster_autonomy.teleop_keyboard:main",
        ],
    },
)
