from setuptools import setup
package_name = 'bip_benchmark'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Harshavardhan Coimbatore Sekar', maintainer_email='harshasekar.09@gmail.com',
    description='Scenario-based benchmarking of the executive layer', license='MIT',
    entry_points={'console_scripts': ['benchmark_node = bip_benchmark.benchmark_node:main']},
)
