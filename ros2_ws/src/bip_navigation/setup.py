import os
from glob import glob
from setuptools import setup

package_name = 'bip_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harshavardhan Coimbatore Sekar',
    maintainer_email='harshasekar.09@gmail.com',
    description='A* + pure pursuit navigation action server (Nav2-free)',
    license='MIT',
    entry_points={'console_scripts': [
        'nav_server = bip_navigation.nav_server:main',
    ]},
)
