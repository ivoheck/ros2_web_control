from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros2_web_control'

# Get all files in frontend folder for building package
frontend_files = glob('frontend/**/*', recursive=True)
frontend_files = [f for f in frontend_files if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/start_web_server.launch.py',
        ]),
        ('share/' + package_name + '/frontend', frontend_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivo',
    maintainer_email='ivoheck99@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'fast_api_node = {package_name}.fast_api:main',
        ],
    },
)