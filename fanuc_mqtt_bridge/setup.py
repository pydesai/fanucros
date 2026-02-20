from setuptools import find_packages, setup

package_name = "fanuc_mqtt_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/bridge_config.yaml"]),
        (f"share/{package_name}/launch", ["launch/fanuc_mqtt_bridge.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML", "paho-mqtt"],
    zip_safe=True,
    maintainer="FANUC CORPORATION",
    maintainer_email="fanuc-ros-maintainer@fanuc.co.jp",
    description="Bridge FANUC telemetry and optional write operations to MQTT.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fanuc_mqtt_bridge = fanuc_mqtt_bridge.bridge_node:main",
        ],
    },
)
