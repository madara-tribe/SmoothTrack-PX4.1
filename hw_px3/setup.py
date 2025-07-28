from setuptools import setup

setup(
    name='hw_px3',
    version='0.0.0',
    packages=['hw_px3'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/hw_px3']),
        ('share/hw_px3', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ESP8266 subscriber to angle',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hw_px3 = hw_px3.main:main'
        ],
    },
)
