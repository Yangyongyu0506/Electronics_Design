from setuptools import find_packages, setup

package_name = 'slam_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            ['launch/slam_bringup.launch.py'],
        ),
        (
            'share/' + package_name + '/config',
            ['config/mapper_params_online_async.yaml'],
        ),
        (
            'share/' + package_name + '/rviz',
            ['rviz/slam.rviz'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuchi',
    maintainer_email='yuchi@localhost',
    description='ESP32 slam-task SLAM bringup',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'translater_node = slam_bringup.translater_node:main',
        ],
    },
)
