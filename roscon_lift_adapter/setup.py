import os
from glob import glob
from setuptools import setup

package_name = 'roscon_lift_adapter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luca Della Vedova',
    maintainer_email='luca@openrobotics.org',
    description='Demo lift adapter to be used with rmf_demos simulations',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lift_adapter = roscon_lift_adapter.lift_adapter:main',
            'lift_manager = roscon_lift_adapter.lift_manager:main'
        ],
    },
)
