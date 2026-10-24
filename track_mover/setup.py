
from setuptools import find_packages, setup

package_name = 'track_mover'

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
    maintainer='reinoxe',
    maintainer_email='reinoxe@example.com',
    description='Menggerakkan mobil melewati track kanan lewat /cmd_vel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover_track_kanan = track_mover.mover_track_kanan:main',
        ],
    },
)
