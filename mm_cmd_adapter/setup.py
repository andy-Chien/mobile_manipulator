from setuptools import setup

package_name = 'mm_cmd_adapter'
submodules = "mm_cmd_adapter/path_adapter"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Chien',
    maintainer_email='N2107687J@ntu.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_adapter_node = scripts.path_adapter_node:main'
        ],
    },
)
