from setuptools import find_packages, setup

package_name = 'temperature_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henrique',
    maintainer_email='henrique@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = temperature_monitor.publisher_node:main',
            'monitor_node = temperature_monitor.temperature_monitor_node:main',
        ],
    },
)
