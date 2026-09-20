from setuptools import find_packages, setup


package_name = "nav2_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/nav2_gateway.launch.py", "launch/nav2_sim.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "gateway = nav2_gateway.gateway:main",
            "simulator_tf = nav2_gateway.tf_bridge:main",
        ],
    },
)
