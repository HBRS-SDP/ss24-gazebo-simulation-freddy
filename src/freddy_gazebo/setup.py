from setuptools import find_packages, setup
import os
import glob

package_name = 'freddy_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob.glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob.glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'scripts'), glob.glob(os.path.join('scripts', '*.py'))),
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
            f'control_publisher = {package_name}.control_publisher:main',
            f'arms_control = {package_name}.arms_control:main',
            f'freddy_gazebo = {package_name}.freddy_gazebo:main',
        ],
    },
)
