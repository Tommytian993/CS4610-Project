from setuptools import setup

setup(
    name="study_delivery",
    version="0.0.1",
    packages=["study_delivery"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/study_delivery"]),
        ("share/study_delivery", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CS4610 Project Team",
    maintainer_email="wang.tianye2@northeastern.edu",
    description="Stretch delivery project scaffold",
    license="TODO",
    entry_points={"console_scripts": ["status = study_delivery.status_node:main",
        "hand_raise = study_delivery.hand_raise_node:main",
        "delivery_dialogue = study_delivery.delivery_dialogue:main",
        "stretch_ack = study_delivery.stretch_ack:main"]},
)
