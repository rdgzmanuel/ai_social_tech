from setuptools import setup

package_name = "stc_camera"

setup(
    name=package_name,
    version="1.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jaime Boal",
    maintainer_email="jboal@comillas.edu",
    description="Control stack for Social Tech Challenge @ Comillas ICAI.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wall_follower = amr_control.wall_follower_node:main",
        ],
    },
)
