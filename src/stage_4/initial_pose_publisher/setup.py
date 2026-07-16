from setuptools import find_packages, setup

package_name = 'initial_pose_publisher'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_dqn_stage4=initial_pose_publisher.initial_pose_dqn_stage4:main',
            'initial_pose_world=initial_pose_publisher.initial_pose_world:main',
        ],
    },
)
