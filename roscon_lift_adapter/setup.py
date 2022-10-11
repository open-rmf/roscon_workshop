from setuptools import setup

package_name = 'roscon_lift_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/sim_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roscon_lift_adapter = roscon_lift_adapter.roscon_lift_adapter:main',
            'simulated_lift = roscon_lift_adapter.simulated_lift:main'
        ],
    },
)
