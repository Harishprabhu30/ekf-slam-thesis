from setuptools import setup
import os
from glob import glob

package_name = "ekf_slam_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        # Install launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),

        # Install config files (yaml)
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),

        # Install rviz configs if you have them
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harish",
    maintainer_email="harishprabhu3007@gmail.com",
    description="EKF/Hybrid SLAM baseline simulation package for Isaac Sim + ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # This maps a ROS2 runnable name -> python module:function
            "encoder_odom_publisher = ekf_slam_sim.encoder_odom_publisher:main",
<<<<<<< HEAD
            "imu_covariance_fixer = ekf_slam_sim.imu_covariance_fixer:main",
=======
            "trajectory_player = ekf_slam_sim.trajectory_player:main",
>>>>>>> main
        ],
    },
)

