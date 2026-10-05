from setuptools import setup
package_name = 'bip_battery'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Harshavardhan Coimbatore Sekar', maintainer_email='harshasekar.09@gmail.com',
    description='Simulated battery with dock charging', license='MIT',
    entry_points={'console_scripts': ['battery_node = bip_battery.battery_node:main']},
)
