from setuptools import find_packages, setup

package_name = 'pallet_detect'

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
    maintainer='virgoyal',
    maintainer_email='virgoyal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "test_node=pallet_detect.node_one:main",
            'inference_node = pallet_detect.inference_node:main',
            'image_viewer = pallet_detect.image_viewer:main'
        ],
    },
)
