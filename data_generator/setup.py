from setuptools import find_packages, setup

package_name = 'data_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/data_launch.py'])
    ],
    install_requires=['setuptools',
                      'rclpy',
                      'numpy'],
    zip_safe=True,
    maintainer='sitong',
    maintainer_email='sitong@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generator = data_generator.cmd_generator:main',
            'recoder = data_generator.recoder:main'

        ],
    },
)
