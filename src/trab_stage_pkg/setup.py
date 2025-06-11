from setuptools import find_packages, setup

package_name = 'trab_stage_pkg'

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
    maintainer='RafaellaGabriel',
    maintainer_email='rafaellaql2002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_node = trab_stage_pkg.main_node:main',
            'publisher = trab_stage_pkg.publisher:main',
            'subscriber = trab_stage_pkg.subscriber:main',
            'robot = trab_stage_pkg.robot:main',

        ],
    },
)
