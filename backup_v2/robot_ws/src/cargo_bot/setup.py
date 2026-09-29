from glob import glob

from setuptools import setup

package_name = "cargo_bot"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cargo Bot Team",
    maintainer_email="team@example.com",
    description="Autonomous cargo robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_driver = cargo_bot.base_driver:main",
            "payload_node = cargo_bot.payload_node:main",
            "speed_limiter = cargo_bot.speed_limiter:main",
            "vision_node = cargo_bot.vision_node:main",
            "mission = cargo_bot.mission:main",
        ],
    },
)
