# -*- coding: utf-8 -*-
from distutils.core import setup

setup(
    name='pypozyx',
    packages=['pypozyx', 'pypozyx.definitions',
              'pypozyx.structures', 'pypozyx.tests'],
    version='1.0.0',
    description='Python library for Pozyx',
    author='Laurent Van Acker',
    author_email='laurent@pozyx.io',
    url = 'https://github.com/pozyxLabs/Pozyx-Python-library',
    download_url='https://github.com/pozyxLabs/Pozyx-Python-library/archive/v1.0.tar.gz',
    keywords=['pozyx', 'serial', 'positioning', 'localisation'],
    classifiers=[
        'Programming Language :: Python',
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: End Users/Desktop',
        'Operating System :: OS Independent',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering',
    ],
    long_description="""\
    Pozyx Python library, used to interface with the Pozyx without need of an arduino.
    Currently supports USB (serial), planned I2C support.

    Both Python 2 and 3 are supported.
    """
)
