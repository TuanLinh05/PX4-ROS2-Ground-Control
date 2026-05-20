from setuptools import setup
import os
from glob import glob

package_name = 'px4_gcs'

# Collect all static files recursively
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
]

# Add static web files
for dirpath, dirnames, filenames in os.walk('static'):
    install_dir = os.path.join('share', package_name, dirpath)
    files = [os.path.join(dirpath, f) for f in filenames]
    if files:
        data_files.append((install_dir, files))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='jun@example.com',
    description='PX4 Ground Control Station Web UI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gcs_node = px4_gcs.gcs_node:main',
        ],
    },
)
