from setuptools import find_packages, setup

package_name = 'chung_object_follower'

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
    maintainer='Nathan Chung',
    maintainer_email='nathanchung@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'finder = chung_object_follower.find_object:main',
            'rotator = chung_object_follower.rotate_object:main'
        ],
    },
)
