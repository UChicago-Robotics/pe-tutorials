from setuptools import setup
from pathlib import Path
from glob import glob
from collections import defaultdict
import os
import fnmatch

package_name = 'wallstopper'

# This section just makes sure our gazebo resources are imported properly.
wildcard_matches = ['*.model', '*.world', '*.config', '*.sdf', '*.dae']
folders_dict = defaultdict(list)

for root, dirnames, filenames in os.walk('.'):
    for wildcard in wildcard_matches:
        for filename in fnmatch.filter(filenames, wildcard):
            target_path = os.path.join(root, filename)
            folders_dict['share/' 
                 + package_name 
                 + '/' 
                 + '/'.join(target_path.split('/')[0:-1])].append(target_path)

matches_out = list(folders_dict.items())

"""
Configuration
"""
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/wallstopper_world.launch.py']),
    ] + matches_out,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='winston',
    maintainer_email='winstonhartnett@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
