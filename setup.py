from setuptools import find_packages, setup

PACKAGE_NAME = "nrai_perception"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools", "opencv-python>=4.12.0.88", "numpy>=2.2.6"],
    zip_safe=True,
    maintainer="Newcastle Racing AI",
    description="Newcastle Racing AI module for perception",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
        "standalone": [
            "nrai_rosutils @ git+https://github.com/NewcastleRacingAI/nrai_rosutils.git",
        ],
        "models": [
            "ultralytics",
        ]
    },
    entry_points={
        "console_scripts": [
            "nrai_perception = nrai_perception.__main__:main",
            "ros_node = nrai_perception.ros:main",
        ],
    },
)
