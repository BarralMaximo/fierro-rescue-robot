from glob import glob

from setuptools import setup

package_name = 'fierro'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Máximo Barral',
    maintainer_email='barralm@udesa.edu.ar',
    description='Búsqueda y rescate en un laberinto con ROS 2 (equipo Fierro, Challenge JAR 2026).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception = fierro.perception:main',
            'localizer = fierro.localizer:main',
            'navigator = fierro.navigator:main',
        ],
    },
)
