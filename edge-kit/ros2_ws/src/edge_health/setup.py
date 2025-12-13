from setuptools import setup
import os
from glob import glob

package_name = 'edge_health'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Awais',
    maintainer_email='awais@example.com',
    description='Physical AI System Monitoring and Control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_monitor = edge_health.health_monitor:main',
            'threshold_monitor = edge_health.threshold_monitor:main',
            'alert_publisher = edge_health.alert_publisher:main',
            'safe_shutdown_service = edge_health.safe_shutdown_service:main',
        ],
    },
)