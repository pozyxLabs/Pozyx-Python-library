# Pozyx-Python-library
A Python library to work with the pozyx indoor positioning system over USB.

This library works with both Python 2 and 3.

## Prerequisites
* Download and install Python. On Windows, make your life easier and make sure Python is in your PATH. A recommended install is therefore the [Anaconda Suite](https://www.anaconda.com/download/) by Continuum. If you're going to follow the tutorials, you'll need to install Python 3 for the python-osc support.
* Install the PySerial package. If you have pip installed, you can do this by writing `pip install pyserial` in your command line interface (cmd on Windows).
* **Windows only** install [ST's virtual COM driver](http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-stm32102.html). After running this installer, please run the correct driver package for your system, located in `C:\Program Files (x86)\STMicroelectronics\Software\Virtual comport driver`. Choose Win7 if you run Windows 7 or older. Choose Win8 for Windows 8 or newer. Run `dpinst_amd64.exe` on a 64-bit system, `dpinst_x86.exe` on a 32-bit system.

## Installing this package
Just run `pip install pypozyx`

PyPozyx is now installed. To check whether it is: if you followed all the steps correctly, and know which port your Pozyx is on, the following code should work:

```python
from pypozyx import PozyxSerial
port = 'COMX' # on UNIX systems this will be '/dev/ttyACMX'
p = PozyxSerial(port)
```

If your port is correct and the serial connection to the Pozyx isn't used by other software, this will run without any errors.

### But! How do I know what port my Pozyx is on?
* You can see the COM ports on your system easily using Python with: `python -c "from pypozyx import *;list_serial_ports()"`

* **NEW** You can quickly find whether there's a recognized Pozyx device using: `python -c "from pypozyx import *;print(get_first_pozyx_serial_port())"`

## Documentation and examples
You can find the Python tutorials on our site: https://docs.pozyx.io/creator/latest/python.

Documentation for the Python library can be found here: https://pypozyx.readthedocs.io.

* This was originally a port of the Pozyx's Arduino library, so most of the [Arduino Library Documentation](https://ardupozyx.readthedocs.io) is transformable to this. The major difference is that you don't ever again need to pass along the length of the data you're reading/writing. This is taken care of by the library through the Data and SingleRegister classes like so:

```python
whoami = SingleRegister()
pozyx.regRead(POZYX_WHO_AM_I, whoami) # which is pozyx.getWhoAmI(whoami)
```
* `SingleRegister(value=0, size=1, signed=1)` is basically an instance `Data([0], 'B')`, which functions as a single uint8_t. If you want to make your custom data, for a single register you can adapt the size and signed parameters, and for larger data structures you can use your own data formats. `Data([0]*3, 'BHI')`, for example, creates a structure of 1 uint8_t, uint16_t and uint32_t. Writing and reading data using this example as a parameter will automatically read/write 7 bytes worth of data. To specify your own data formats, check the [struct package documentation for Python 3](https://docs.python.org/3.5/library/struct.html#format-characters) or [Python 2](https://docs.python.org/2/library/struct.html).

* A more pythonic library would be nice, but isn't in the works.


More usage examples can be found in the [useful](https://github.com/pozyxLabs/Pozyx-Python-library/tree/master/useful) and [tutorials](https://github.com/pozyxLabs/Pozyx-Python-library/tree/master/tutorials) folders of the repository.
