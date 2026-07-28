from setuptools import find_packages, setup

package_name = 'monitor_sys'

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
            'system_monitor = monitor_sys.monitor_pjt.system_monitor:main',
            'dashboard = monitor_sys.monitor_pjt.dashboard:main',
            'web_ui = monitor_sys.monitor_pjt.web_ui:main',
        ],
    },
)
