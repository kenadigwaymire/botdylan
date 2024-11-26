from setuptools import find_packages, setup
from glob import glob

package_name = 'botdylan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['botdylan', 'botdylan.*'], exclude=['test']),  # Include all submodules in botdylan
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),  # Include meshes if applicable
        ('share/' + package_name + '/code', glob('code/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Bot Dylan Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory = botdylan.trajectory:main',  # Add trajectory executable
        ],
    },
)
