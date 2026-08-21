from setuptools import find_packages, setup

package_name = 'dooy_spiral_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/monitor.launch.py',
            'launch/web_admin.launch.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'fastapi>=0.95',
        'uvicorn>=0.20',
    ],
    zip_safe=True,
    maintainer='kimdooyong',
    maintainer_email='kimdooyong@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hand_drip_pour = dooy_spiral_monitor.kettle_circle_pour:main',
            'kettle_circle_pour_v2 = dooy_spiral_monitor.kettle_circle_pour_v2:main',
            'kettle_circle_pour_v3 = dooy_spiral_monitor.kettle_circle_pour_v3:main',
            'kettle_circle_pour_v4 = dooy_spiral_monitor.kettle_circle_pour_v4:main',
            'kettle_circle_pour_test = dooy_spiral_monitor.kettle_circle_pour_test:main',
            'path_viz = dooy_spiral_monitor.path_viz:main',
            'hand_drip_teach = dooy_spiral_monitor.teach_helper:main',
        ],
    },
)
