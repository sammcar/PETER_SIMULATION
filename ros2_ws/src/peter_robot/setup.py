from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'peter_robot'

def get_model_data_files():
    paths = []
    for dirpath, dirnames, filenames in os.walk('models'):
        for file in filenames:
            src_path = os.path.join(dirpath, file)
            dst_path = os.path.join('share', package_name, dirpath, file)
            paths.append((os.path.dirname(dst_path), [src_path]))
    return paths

def only_files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), only_files('urdf/*')),
        (os.path.join('share', package_name, 'launch'), only_files('launch/*')),
        (os.path.join('share', package_name, 'src'), only_files('src/*.py')),
        (os.path.join('share', package_name, 'meshes'), only_files('meshes/*')),
        (os.path.join('share', package_name, 'worlds'), only_files('worlds/*')),
        (os.path.join('share', package_name, 'config'), only_files('config/*')),
        (os.path.join('share', package_name, 'docs'), only_files('docs/*')),
        (os.path.join('share', package_name, 'scripts'), only_files('scripts/*')),
    ] + get_model_data_files(),
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
            'peter_teleop_stdin = src.peter_teleop_stdin:main',
            'peter_test_joints = src.test_move_joints:main',
            'red_neuronal = src.red_neuronal:main',
            'camera_node = src.camera_node:main',
            'camera_Adjust = src.camera_Adjust:main',
            'plotter = src.plotter:main',
            'raster_graphic_node = src.raster_graphic_node:main',
            'live_signals = src.live_signals:main',
            'neural_recorder = src.neural_recorder:main',
            'peter_stability_monitor = src.peter_stability_monitor:main',
            'RobustnessMetric = src.robustnessListener:main',
            "RMSE_node = src.RMSE_node:main",
            'test_manager = scripts.test_manager:main',
            'metrics_recorder = src.metrics_recorder:main',
        ],
    },
)