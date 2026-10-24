from glob import glob

from setuptools import setup

paquete = "go2_nav"
setup(
    name=paquete,
    version="1.9.0",
    packages=[paquete, paquete + ".nucleo"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + paquete]),
        ("share/" + paquete, ["package.xml"]),
        ("share/" + paquete + "/launch", glob("launch/*.py")),
        ("share/" + paquete + "/config", glob("config/*")),
        ("share/" + paquete + "/rviz", glob("rviz/*")),
        ("share/" + paquete + "/urdf", glob("urdf/*")),
        ("share/" + paquete + "/meshes", glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="korhan",
    maintainer_email="korhan@example.com",
    description="Navegacion autonoma Go2 (ROS 2)",
    license="MIT",
    entry_points={"console_scripts": [
        "driver = go2_nav.driver:main",
        "limpiador = go2_nav.limpiador:main",
        "gestor = go2_nav.gestor:main",
        "escaleras = go2_nav.escaleras_nodo:main",
        "vision = go2_nav.vision:main",
        "perro_sim = go2_nav.perro_sim:main",
        "ps4 = go2_nav.ps4:main",
        "mapa3d = go2_nav.mapa3d:main",
        "slam3d = go2_nav.slam3d:main",
        "nube = go2_nav.nube_cruda:main",
        "imu = go2_nav.imu:main",
        "go2 = go2_nav.cli:main",
    ]},
)
