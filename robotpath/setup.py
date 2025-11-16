import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robotpath'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sahilsnair',
    maintainer_email='sahil.nair@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_plan = robotpath.motion_plan:main',
            'collision = robotpath.motion_collision:main',
            'fk_service = robotpath.fk_service:main',
            'object = robotpath.collision_object:main',
            'alternate_object = robotpath.collision_object_alternative:main',
        ],
    },
)
