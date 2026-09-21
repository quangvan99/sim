from setuptools import setup


package_name = "nav2_gateway"

setup(
    name=package_name,
    version="0.1.0",
    py_modules=["gateway"],
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
            "gateway = gateway:main",
            "simulator_tf = gateway:main_tf",
        ],
    },
)
