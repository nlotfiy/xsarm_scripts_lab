import glob
from setuptools import find_packages, setup

package_name = 'xsarm_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nima',
    maintainer_email='nima.st36@gmail.com',
    description='Custom scripts for controlling the Interbotix VX300S arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fk_script = xsarm_scripts.fk_script:main',
            'lab_template = xsarm_scripts.lab_template:main',
            'bartender_supervised = xsarm_scripts.bartender_supervised:main',
            'elbow_range_demo = xsarm_scripts.elbow_range_demo:main',
            'work_envelope = xsarm_scripts.work_envelope:main',
            'park_arm = xsarm_scripts.park_arm:main',
            'driver_guard = xsarm_scripts.driver_guard:main',
            'estop = xsarm_scripts.estop:main',
            'envelope_server = xsarm_scripts.envelope_server:main',
            'supervised_demo = xsarm_scripts.supervised_demo:main',
        ],
    },
)
