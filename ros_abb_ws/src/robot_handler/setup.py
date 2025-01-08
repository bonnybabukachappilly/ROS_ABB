import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (
            f'share/{package_name}', ['package.xml']
        ),
        (
            f'share/{package_name}', ['.env']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bonnybk',
    maintainer_email='bonnybabukachappilly@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websocket_handler = robot_handler.ws:main',
            'api_service_handler = robot_handler.api:main',
        ],
    },
)
