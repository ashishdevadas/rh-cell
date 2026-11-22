from glob import glob

from setuptools import setup

package_name = "rh_cell_ic"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/plc", glob("plc/*")),
        ("share/" + package_name + "/phoebus", glob("phoebus/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ashish Devadas",
    maintainer_email="ashishdevadas00@gmail.com",
    description="EPICS IOC, PLC interlock bridge and operator screen for the remote-handling cell",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "plc_bridge = rh_cell_ic.plc_bridge:main",
            "epics_ioc = rh_cell_ic.epics_ioc:main",
            "estop_demo = rh_cell_ic.estop_demo:main",
        ],
    },
)
