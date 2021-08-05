from setuptools import setup
import os
from glob import glob
package_name = 'tracking_database'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/database.launch.xml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/devices.yaml')),
        (os.path.join('share', package_name, 'resource'),
            glob('resource/output.json'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='judith',
    maintainer_email='judithlimlj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'database_interface = tracking_database.database_interface:main',
            'rtls_check = tracking_database.rtls_check:main',
            'restful_interface = tracking_database.restful_interface:main'
        ],
    },
)
