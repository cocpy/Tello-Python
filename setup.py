import setuptools

with open('README.md', 'r') as fh:
    long_description = fh.read()

setuptools.setup(
    name='tello-python',
    version='1.0.0',
    author='C灵C',
    author_email='c0c@cocpy.com',
    description='Control DJI Tello drone with Python 3',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/cocpy/Tello-Python',
    packages=setuptools.find_packages(),
    install_requires=[
        'opencv-python'
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
)