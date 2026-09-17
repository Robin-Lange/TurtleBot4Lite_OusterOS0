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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobinLange",
    maintainer_email="rl11732@georgiasouthern.edu",
    description="Stationary diagnostics and frame setup for a Create 3 with Ouster OS0.",
    license="Unspecified",
    entry_points={
        "console_scripts": [
            "stationary_diagnostics = tb4_ouster_autonomy.stationary_diagnostics:main",
        ],
    },
)
