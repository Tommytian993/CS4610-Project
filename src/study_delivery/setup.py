from setuptools import setup


package_name = "study_delivery"


setup(
    name=package_name,
    version="0.0.1",

    packages=[package_name],

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/study_delivery"]
        ),
        (
            "share/study_delivery",
            ["package.xml"]
        ),
    ],

    install_requires=["setuptools"],

    zip_safe=True,

    maintainer="CS4610 Project Team",

    maintainer_email="wang.tianye2@northeastern.edu",

    description="Stretch student-to-student delivery project",

    license="TODO",

    entry_points={
        "console_scripts": [
            "status = study_delivery.status_node:main",

            "hand_raise = study_delivery.hand_raise_node:main",

            "delivery_dialogue = study_delivery.delivery_dialogue:main",

            "stretch_ack = study_delivery.stretch_ack:main",

            "handover_gesture = study_delivery.handover_gesture_node:main",

            "pickup_controller = study_delivery.pickup_controller:main",
        ],
    },
)