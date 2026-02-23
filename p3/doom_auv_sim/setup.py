from setuptools import find_packages, setup

package_name = 'doom_auv_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['doom_auv_sim/rviz/doom_sim.rviz']),
        ('share/' + package_name + '/launch', ['launch/doom_sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergio',
    maintainer_email='al428707@uji.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'doom_sim_node = doom_auv_sim.nodes.main_sim_node:main',
            'teleop_node = doom_auv_sim.nodes.teleop_node:main',
            'acoustic_channel_node = doom_auv_sim.nodes.acoustic_channel_node:main',
            'mission_node = doom_auv_sim.nodes.mission_node:main',
            'arbitrator_node = doom_auv_sim.nodes.arbitrator_node:main',
            'nav2_bridge = doom_auv_sim.nav2_bridge:main',
            'viz_bridge_node = doom_auv_sim.nodes.viz_bridge_node:main',
        ],
    },
)
