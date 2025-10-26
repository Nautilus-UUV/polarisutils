from setuptools import setup, find_packages

package_name = 'polarisutils'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/examples', ['examples/basic_usage.py']),
        ('share/' + package_name + '/examples', ['examples/migration_example.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nautilus Software Team',
    maintainer_email='',
    description='ROS2 interface management system for Polaris UUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any console scripts here if needed
        ],
    },
)