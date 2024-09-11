from setuptools import find_packages, setup

package_name = 'botafogo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tiquinho.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matheus Pereira',
    maintainer_email='uniemathrodrigues@fei.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'savarino = botafogo.savarino:main',
            'almada = botafogo.almada:main',
        ],
    },
)
