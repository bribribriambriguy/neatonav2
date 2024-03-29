from setuptools import setup
import os
from glob import glob

package_name = 'neatonav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share',package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share',package_name, 'map'), glob('map/*')),
        (os.path.join('share',package_name, 'config'), glob('config/*')),
        (os.path.join('share',package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share',package_name, 'srv'), glob('srv/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brian',
    maintainer_email='bkletchikov@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'neato_node = neatonav2.neato_node:main',
                'neato_driver = neatonav2.neato_driver:main'
        ],
        
    },
)
