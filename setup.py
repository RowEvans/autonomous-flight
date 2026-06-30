from setuptools import find_packages, setup

package_name = 'cessna_offboard'

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
    maintainer='rowan',
    maintainer_email='rowan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher = cessna_offboard.publishing_node:main',
            'listener = cessna_offboard.listener_node:main',
            'px4 = cessna_offboard.px4_listener:main',
            'offboard = cessna_offboard.offboard_node:main',
        ],
    },
)
