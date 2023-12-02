import os

from glob import glob
from setuptools import setup


package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'),glob(os.path.join('launch/*.launch.py'))),
        (os.path.join('share', package_name,'config'),glob(os.path.join('config/*.yaml'))),
        (os.path.join('share', package_name,'description'),glob(os.path.join('description/*.xacro'))),
        (os.path.join('share', package_name,'description'),glob(os.path.join('description/*.srdf'))),
        (os.path.join('share', package_name,'description'),glob(os.path.join('description/*.urdf'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description','xacro', '*.xacro'))),
        (os.path.join('share', package_name, 'description', 'mesh'), glob(os.path.join('description','mesh', '*.STL'))),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joas329',
    maintainer_email='joaquinphilco@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drivePID = my_bot.drivePID:main"
            "velPID = my_bot.VelPID:main",
            "effortPID = my_bot.EffortPID:main",
            "PID = my_bot.PID:main",
            "udp_conv = my_bot.udp_conv:main"
        ],
    },
)
