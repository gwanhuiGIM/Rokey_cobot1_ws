from setuptools import find_packages, setup

package_name = 'rokey'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/monitor.launch.py']),
    ],
    install_requires=['setuptools'],
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
            'simple_move = rokey.move:main',
            'plate_monitor = rokey.plate_monitor:main',
            'F_3_drl_sun_gear = rokey.F_3_drl_sun_gear:main',
            'plate_monitor_v2 = rokey.plate_monitor_v2:main',
            'plate_monitor_v3 = rokey.plate_monitor_v3:main',
            'plate_balance_controller = rokey.plate_balance_controller:main',
            'plate_move = rokey.plate_move:main',
            'control_GUI = rokey.control_GUI:main',
            'plate_pose_gui = rokey.plate_pose_invariant_gui:main',
            'contact_stop = rokey.detect.contact_stop:main',
            'move_until_contact = rokey.detect.move_until_contact:main',
            'detection_v1 = rokey.detect.detection_v1:main',
            'inference_shape = rokey.detect.inference_shape:main',
            'probe_grip_v1 = rokey.detect.probe_grip_v1:main',
            'probe_grip_sim = rokey.detect.probe_grip_sim:main',
            'probe_grip_sim_v2 = rokey.detect.probe_grip_sim_v2:main',
            'probe_grip_sim_v3 = rokey.detect.probe_grip_sim_v3:main',
            'probe_grip_v2 = rokey.detect.probe_grip_v2:main',
            'hand_drip_pour = rokey.hand_drip.kettle_circle_pour:main',
            'kettle_circle_pour_v2 = rokey.hand_drip.kettle_circle_pour_v2:main',
            'hand_drip_teach = rokey.hand_drip.teach_helper:main',
            'system_monitor = rokey.monitor_pjt.system_monitor:main',
            'dashboard = rokey.monitor_pjt.dashboard:main',
        ],
    },
)
