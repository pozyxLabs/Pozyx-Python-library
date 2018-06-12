pypozyx
========

About
-----

pypozyx is a Python library made for providing an interface with a Pozyx device over a serial connection. This serial interface then also allows communication with remote Pozyx devices through the connected device.

The library was largely inspired by the already existing Arduino library, yielding a lot of advantages that carried over from the Arduino library. However, as a result pypozyx is not that Pythonic. It is our intention to improve this with our next big release.

Features
--------

* Easy to use, allowing both high-level and low-level interfacing with the Pozyx device.
* Uses the excellent pySerial library for cross-platform functionality.
* Works with Python 2 and Python 3
* Pozyx device serial port detection.
* Specialized data structures for all important Pozyx data.

Requirements
------------

* Python 2.7 or newer, or Python 3.4 and newer
* pySerial > 3.0

Pozyx serial connection requirements
------------------------------------

Linux
~~~~~

* Make sure you have permissions set to use the serial device.
* To get permission once (given your device is on port ACM0) you can run ``sudo chmod 666 /dev/ttyACM0``
* To get permission forever you can run ``sudo adduser $USER dialout`` or the relevant group for your distro.

MacOS
~~~~~

* Normally everything should work out of the box.

Windows
~~~~~~~

* Please use Windows 7 or newer.
* Install `STM32 Virtual COM Port Driver <http://www.st.com/en/development-tools/stsw-stm32102.html>`_.

.. note::

   After running this installer, you have to install the correct driver package for your system.

   The driver installers are located in *C:\\Program Files (x86)\\STMicroelectronics\\Software\\Virtual comport driver*.

   Choose Win7 if you run Windows 7 or older. Choose Win8 for Windows 8 or newer. Run "dpinst_amd64.exe" on a 64-bit system, "dpinst_x86.exe"on a 32-bit system.

Installation
------------

Currently, pypozyx is easily installable from `Python Package index (PyPi) <https://pypi.org/>`_, but can also be installed from source.

From PyPi
~~~~~~~~~

To install the library from `PyPi <https://pypi.org/project/pypozyx/>`_, you can run either of the following:

*  ``pip install pypozyx`` or ``python -m pip install pypozyx``.
*  ``pip3 install pypozyx`` or ``python3 -m pip install pypozyx``.

.. note::

   If installation fails due to permission issues, or the package doesn't seem to install, please try to add --user as a flag to pip, ``pip install --user pypozyx``, or use sudo ``sudo pip install pypozyx``.

From source
~~~~~~~~~~~

To install from source, you'll have to download the source files first, of course. You can do this either by:

* ``git clone https://github.com/pozyxLabs/Pozyx-Python-library``
* Download it from `the repository <https://github.com/pozyxLabs/Pozyx-Python-library>`_ and extracting it.
* Downloading the `zip file <https://github.com/pozyxLabs/Pozyx-Python-library/archive/master.zip>`_ directly, and extracting it.

Then, in your extracted/downloaded folder, run ``python setup.py install`` or ``python3 setup.py install``.
