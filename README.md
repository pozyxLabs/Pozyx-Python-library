# Pozyx-Python-library
The unofficial release of the Python library (Beta version) to work with the pozyx indoor positioning system

This library requires Python 3. A Python 2 version will be made available separately.

### Prerequisites:
* Download and install Python 3, on Windows, make your life easier and make sure Python is in your PATH. A recommended install is therefore the [Anaconda Suite](https://www.continuum.io/downloads) by Continuum.
* Install the PySerial package. If you have pip installed, you can do this by writing `pip install pyserial` in your command line interface (cmd on Windows). Pip is automatically installed if you installed the Anaconda Suite.
* **Windows only** install [ST's virtual COM driver](http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-stm32102.html). Be careful, this puts the actual installer in your Program Files, so you'll have to go to the STMicroElectronics folder in your Program Files and install the right installer there (if on Windows 10, use the Windows 8 installer)

### Installing this package.
As it's not yet available on PyPi, you will have to install the library from source. This is, however, very easy.
* Download the library as a zip file, or clone it in a folder.
* After changing your command window's working directory to the extracted/cloned folder, perform `python setup.py install`

Ta-da! PyPozyx is now installed.

 
### Is it really?
Yes! It definitely should be. Not convinced? If you followed all the steps correctly, and know which port your Pozyx is on, the following code should work:

```python
from pypozyx import *
port = 'COMX' # on UNIX systems this will be '/dev/ttyACMX'
p = PozyxSerial(port)
```
If your port is correct and the serial connection to the Pozyx isn't used by other software, this will run without any errors.

#### But! How do I know what port my Pozyx is on?
You can use existing software to directly monitor and interface with the Pozyx serially, but then again, why did you install this library? You can see the COM ports on your system easily using Python with:
`python -c "import serial.tools.list_ports;print(serial.tools.list_ports.comports()[0])"`
The ``[0]`` index lists the first detected COM port. If you have multiple serial devices attached, such as Arduino, you can use a higher index until you find the "Pozyx Virtual ComPort in FS Mode" descriptor.


### Documentation and examples
You might notice the current lack of documentation and examples that use this library! This is being worked on! For now, these pointers and pages should be very helpful:
* All functions that exist in the Arduino library, also exist in the Python library under the same name and functionality, so most of the [Arduino Library Documentation](https://www.pozyx.io/Documentation/Datasheet/arduino) is transformable to this. The difference however, is that you don't ever again need to pass along the length of the data you're reading/writing. This is taken care of by the library:
* The Data and SingleRegister classes take care of this. eg. to read out the WhoAmI register, appending to the test code above.    
```python
whoami = SingleRegister()
pozyx.regRead(POZYX_WHO_AM_I, whoami) # which is pozyx.getWhoAmI(whoami)
```
* `SingleRegister(value=0, size=1, signed=1)` is basically an instance `Data([0], 'B')`, which functions as a single uint8_t. If you want to make your custom data, for a single register you can adapt the size and signed parameters, and for larger data structures you can use your own data formats. `Data([0]*3, 'BHI')`, for example, creates a structure of 1 uint8_t, uint16_t and uint32_t. Writing and reading data using this example as a parameter will automatically read/write 7 bytes worth of data. To specify your own data formats, check the [struct package documentation](https://docs.python.org/3.5/library/struct.html#format-characters).
 
* Examples and tutorials source code will be put on the [Pozyx website](http://pozyx.io/) eventually, but will also appear as source code on GitHub.
