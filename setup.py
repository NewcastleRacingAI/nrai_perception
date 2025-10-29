from setuptools import find_packages, setup

PACKAGE_NAME = 'nrai_perception'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Newcastle Racing AI',
    description='Newcastle Racing AI module for perception',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nrai_perception = nrai_perception.__main__:main',
            'ros_node = nrai_perception.ros:main',
        ],
    },
)
