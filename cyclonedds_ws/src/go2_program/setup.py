from setuptools import find_packages, setup

package_name = 'go2_program'

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
    maintainer='soyo',
    maintainer_email='soyo@todo.todo',
    description='Go2 最简前进控制',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'go2_program_node = go2_program.go2_program:main',
            'keyboard_node = go2_program.keyboard_node:main',
        ],
    },
)
