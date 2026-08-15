from glob import glob
from setuptools import find_packages, setup


package_name = "drone_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dimitris-gkagkanakis",
    maintainer_email="jim.gaganakis@gmail.com",
    description="Local web dashboard for interactive swarm operation and monitoring.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dashboard_node = drone_dashboard.dashboard_node:main",
        ],
    },
)
