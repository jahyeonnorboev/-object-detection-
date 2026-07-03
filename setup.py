from setuptools import setup
package_name = 'human_following'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ns2022427138',
    maintainer_email='jk6871@gmail.com',
    description='YOLO based human following package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'last_following = human_following.last_following:main',
            'voice_node = human_following.voice_node:main',
        ],
    },
)
