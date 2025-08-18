from setuptools import find_packages, setup

package_name = 'robot_state_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pnd-humanoid',
    maintainer_email='m14740557520@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_subscriber = robot_state_subscriber.robot_state_subscriber_node:main',
        ],
    },
)
