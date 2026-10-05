from setuptools import setup

package_name = 'bip_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harshavardhan Coimbatore Sekar',
    maintainer_email='harshasekar.09@gmail.com',
    description='ROS-free core logic: grid, A*, behavior tree, FSM, utility, scheduler',
    license='MIT',
    tests_require=['pytest'],
)
