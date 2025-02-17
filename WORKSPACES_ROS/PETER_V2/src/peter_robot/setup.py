from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'peter_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'urdf'),glob('urdf/*')),
        (os.path.join('share', package_name,'launch'),glob('launch/*.py')),
        (os.path.join('share', package_name,'src'),glob('src/*.py')),
        (os.path.join('share', package_name,'meshes'),glob('meshes/*')),
        (os.path.join('share', package_name,'worlds'),glob('worlds/*.world')),
        (os.path.join('share', package_name,'config'),glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='samm.car23@gmail.com',
    description='Peter Robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'peter_controller = src.peter_controller:main',
            'peter_teleop_keyboard = src.peter_teleop_keyboard:main',
            'red_parcial = src.red_parcial:main',
        ],
    },
)
