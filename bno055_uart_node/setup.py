from setuptools import setup

package_name = 'bno055_uart_node'

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
    maintainer='kody',
    maintainer_email='your@email.com',
    description='BNO055 UART IMU node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_uart_node = bno055_uart_node.bno055_uart_node:main',
        ],
    },
)
