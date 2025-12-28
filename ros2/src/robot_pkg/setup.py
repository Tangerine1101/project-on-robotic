from setuptools import setup
import os
from glob import glob

package_name = 'pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # --- ADD THIS: The Config Folder ---
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Keep your init folder for weights
        (os.path.join('share', package_name, 'init'), glob('pkg/init/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tangerin',
    maintainer_email='lazyshiji@gmail.com',
    description='Robot Arm Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # FORMAT: 'command_name = package_name.script_name:main'
            'vision = pkg.vision:main',
            'planner = pkg.planner_node:main', 
            'driver = pkg.serial_driver:main',
        ],
    },
)