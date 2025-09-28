from setuptools import setup

package_name = 'rdj2025_rqt_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='rqt camera plugin for snapshots/recording and publishing to /image',
    license='Apache-2.0',
    entry_points={
        'console_scripts': []
    },
)
