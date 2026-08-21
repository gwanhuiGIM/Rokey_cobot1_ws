from setuptools import find_packages, setup

package_name = 'tray_balance_kkh'

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
    description='M0609 + RG2: TCP 힘/토크 피드백으로 쟁반을 기울여 판 위 물체의 중심을 유지',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tray_balance = tray_balance_kkh.tray_balance:main',
            'tray_balance_phantom = tray_balance_kkh.tray_balance_phantom:main',
        ],
    },
)
