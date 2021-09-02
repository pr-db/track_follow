from setuptools import setup

package_name = 'track_follow'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hewhoshallnotbenamed',
    maintainer_email='hewsnbn@gmail.com',
    description='Line follower for robo Gazebo simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = track_follow.track_follow:main',
        ],
    },
)
