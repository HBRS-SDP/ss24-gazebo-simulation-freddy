from setuptools import find_packages, setup

package_name = 'freddy_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/load_world_and_robot.launch.py']),
        ('share/' + package_name + '/launch', ['launch/freddy_gazebo.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/my_world.sdf']),
        ('share/' + package_name + '/config', ['config/base_controller.yaml']),
        ('share/' + package_name + '/config', ['config/torso_controller.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/freddy.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shinas',
    maintainer_email='shinasshaji12@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
