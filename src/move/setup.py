from setuptools import find_packages, setup

package_name = 'move'

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
    maintainer='kimkh',
    maintainer_email='sbzmf1@o.cnu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_move = move.move:main',
            'gear_move = move.gear_move:main',
            'tray_balance = move.tray_balance:main',
            'tray_balance_phantom = move.tray_balance_phantom:main',
            'tray_balance_debug = move.tray_balance_debug:main',
            'tray_balance_viz = move.tray_balance_viz:main',
            'tray_balance_sim = move.tray_balance_sim:main',
            'accel_estimation = move.natural_point_accel_estimation:main',
            'weight_change_monitor = move.weight_change_monitor:main',
            'contact_grasp = move.contact_grasp:main',
            'plate_monitor = move.plate_monitor:main',
            'simple_input = move.simple_input:main'
        ],
    },
)
