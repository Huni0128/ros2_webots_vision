from setuptools import setup

package_name = 'pyqt_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='',
    author_email='',
    description='ROS2 + PyQt5 GUI for camera & joint visualization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = pyqt_gui.gui_node:main',
        ],
    },
)
