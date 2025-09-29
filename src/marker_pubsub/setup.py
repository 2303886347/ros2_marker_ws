from setuptools import find_packages, setup

package_name = 'marker_pubsub'

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
    maintainer='ubuntu2204',
    maintainer_email='ubuntu2204@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            
            'marker_publisher = marker_pubsub.marker_publisher:main',  # 添加这一行
            'tf_broadcaster = marker_pubsub.tf_broadcaster:main',
            'service_marker_publisher = marker_pubsub.service_marker_publisher:main'
            #前面的marker_publisher 是你在运行时将要用来调用的命令。
            #中间的marker_pubsub.marker_publisher 是模块路径,
            #其中，marker_pubsub 是包名，marker_publisher 是你创建的 Python 文件名（不带 .py 后缀）。
            #最后的main 是在 marker_publisher.py 文件中定义的主入口函数。
        ],
    },
)
