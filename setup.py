from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros2_web_control'

# Get all files in frontend folder for building package
frontend_src = 'frontend/dist'
frontend_dest = os.path.join('share', package_name, 'frontend/dist')

frontend_files = [
    (os.path.join(frontend_dest, os.path.relpath(os.path.dirname(f), frontend_src)), [f])
    for f in glob(f'{frontend_src}/**/*', recursive=True) if os.path.isfile(f)
]

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
        ('share/' + package_name + '/config', [
            'config/web_control_config.yaml',
        ]),
    ] + frontend_files,
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