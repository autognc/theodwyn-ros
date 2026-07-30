from setuptools import setup

package_name = 'theo_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theodwyn-ros maintainers',
    maintainer_email='todo@example.com',
    description="Bridges scrat2_movement's should_record signal to theo_msgs/TheoCode",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broker_bridge_node = theo_bridge.broker_bridge_node:main',
        ],
    },
)
