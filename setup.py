# -*- coding: utf-8 -*-
from os import path
from setuptools import setup
from codecs import open

from pypozyx import VERSION as PYPOZYX_VERSION

here = path.abspath(path.dirname(__file__))
# Get the long description from the README file
with open(path.join(here, 'README.rst'), encoding='utf-8') as readme:
    long_description = readme.read()

setup(
    name='pypozyx',
    packages=['pypozyx', 'pypozyx.definitions', 'pypozyx.tools',
              'pypozyx.structures'],
    version=PYPOZYX_VERSION,
    description='Python library for Pozyx devices',
    install_requires=[
        'pyserial>=3.0'
    ],
    long_description=long_description,
    author='Laurent Van Acker',
    license='GPLv3',
    author_email='laurent@pozyx.io',
    url='https://github.com/pozyxLabs/Pozyx-Python-library',
    download_url='https://github.com/pozyxLabs/Pozyx-Python-library/archive/v{}.tar.gz'.format(PYPOZYX_VERSION),
    keywords=['pozyx', 'serial', 'positioning', 'localisation'],
    classifiers=[
        'Programming Language :: Python',
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: End Users/Desktop',
        'Operating System :: OS Independent',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering',
    ],

)
