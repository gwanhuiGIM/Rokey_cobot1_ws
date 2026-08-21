from setuptools import find_packages, setup

package_name = 'cup_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimdooyong',
    maintainer_email='kimdooyong@todo.todo',
    description='X-직진 접촉 탐지 + 측면 파지 모듈 (probe_grip_v2)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'probe_grip_v2 = cup_detect.probe_grip_v2:main',
            'probe_grip_v3 = cup_detect.probe_grip_v3:main',
            'probe_grip_v4 = cup_detect.probe_grip_v4:main',
        ],
    },
)
