from setuptools import find_packages, setup

package_name = 'ast_testing'

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
    maintainer='anudeep',
    maintainer_email='anoodeep07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = ast_testing.test:main',
            'battery_monitor = ast_testing.battery_monitor:main',
            'behaviour_tree = ast_testing.behaviours:main',
            'state_machine = ast_testing.sm_test:main',
        ],
    },
)
