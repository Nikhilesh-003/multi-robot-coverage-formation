from glob import glob

from setuptools import find_packages, setup

package_name = 'mrc_fleet'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhilesh Kumar',
    maintainer_email='110115553+Nikhilesh-003@users.noreply.github.com',
    description='Fleet coordination: pose sharing, coverage coordinator, formation control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_monitor = mrc_fleet.fleet_monitor:main',
            'coverage_coordinator = mrc_fleet.coverage_coordinator:main',
            'formation_controller = mrc_fleet.formation_controller:main',
        ],
    },
)
