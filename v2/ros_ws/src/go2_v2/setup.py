from glob import glob

from setuptools import setup

setup(
    name="go2_v2",
    version="2.0.0",
    packages=["go2_v2"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/go2_v2"]),
        ("share/go2_v2", ["package.xml"]),
        ("share/go2_v2/launch", glob("launch/*")),
        ("share/go2_v2/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="korhan",
    maintainer_email="info@velkine.com",
    description="Go2 v2 sobre el stack de CMU",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "sensores = go2_v2.sensores:main",
        "guardian = go2_v2.guardian:main",
        "herramientas = go2_v2.herramientas:main",
    ]},
)
