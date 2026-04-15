from setuptools import setup

package_name = 'waypoint_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Waypoint follower',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_follower = waypoint_follower.waypoint_follower:main',
        ],
    },
)
