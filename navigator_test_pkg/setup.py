from setuptools import find_packages, setup

package_name = 'navigator_test_pkg'

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
    maintainer='taker',
    maintainer_email='kurokku34@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_test = navigator_test_pkg.navigator_test:main',
            'send_coordinate = navigator_test_pkg.send_coordinate:main',
        ],
    },
)
