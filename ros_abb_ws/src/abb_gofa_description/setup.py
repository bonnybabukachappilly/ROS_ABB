from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'abb_gofa_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))
        ),
        (
            os.path.join('share', package_name, 'meshes/collision'),
            glob('meshes/collision/*')
        ),
        (
            os.path.join('share', package_name, 'meshes/visual'),
            glob('meshes/visual/*')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')
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
        ],
    },
)