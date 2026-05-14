from setuptools import find_packages, setup


package_name = "rhw_model_scheduler"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Standalone ROS2 model scheduler service.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rhw_model_scheduler_node = rhw_model_scheduler.model_scheduler_node:main",
            "rhw_model_scheduler_smoke = rhw_model_scheduler.smoke_client:main",
            "rhw_model_latency_benchmark = rhw_model_scheduler.latency_benchmark:main",
            "rhw_model_export_tensorrt = rhw_model_scheduler.export_tensorrt:main",
        ]
    },
)
