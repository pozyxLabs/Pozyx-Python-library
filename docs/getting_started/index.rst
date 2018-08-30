Getting started
---------------

Finding your serial port
~~~~~~~~~~~~~~~~~~~~~~~~

There's a helper in the library for identifying the first serial port that is a Pozyx. This is easily done with a Python snippet

.. TODO link to the function in the docs.

.. code-block:: python

   import pypozyx

   print(pypozyx.get_first_pozyx_serial_port())

Or from the command line

``python -c "from pypozyx import *;print(get_first_pozyx_serial_port())"``

If there is no Pozyx device recognized, the function will return ``None`` and thus nothing will be printed.


Connecting to the Pozyx
~~~~~~~~~~~~~~~~~~~~~~~

Connecting with the Pozyx is very straightforward. A safe way is presented here:

.. code-block:: python

   from pypozyx import PozyxSerial, get_first_pozyx_serial_port

   serial_port = get_first_pozyx_serial_port()

   if serial_port is not None:
       pozyx = PozyxSerial(serial_port)
       print("Connection success!")
   else:
       print("No Pozyx port was found")

With this, you have a pozyx object with the full API at your fingertips. For example, you can read the `UWB settings <https://www.pozyx.io/Documentation/Tutorials/uwb_settings>`_ with the following snippet.

.. code-block:: python

   from pypozyx import PozyxSerial, get_first_pozyx_serial_port, UWBSettings

   serial_port = get_first_pozyx_serial_port()

   if serial_port is not None:
       pozyx = PozyxSerial(serial_port)
       uwb_settings = UWBSettings()
       pozyx.getUWBSettings(uwb_settings)
       print(uwb_settings)
   else:
       print("No Pozyx port was found")


General philosophy
~~~~~~~~~~~~~~~~~~

As said in the introduction, the pypozyx library was heavily inspired by the Arduino library, making it less pythonic.

* The functions are camelCased
* Almost all functions return a status and take the relevant data container as an argument.

This had as an advantage that users coming from Arduino could easily adapt their code, and that the documentation was very similar. However, I'd love to change these things when I make a 2.0 release.

Essentially, you can do three things with Pozyx:

1. Reading register data, which includes sensors and the device's configuration
2. Writing data to registers, making it possible to change the device's configuration ranging from its positioning algorithm to its very ID.
3. Performing Pozyx functions like ranging, positioning, saving the device's configuration to its flash memory...

All these things are possible to do on the device connected to your computer, and powered remote devices as well. In this section we'll go over all of these.

Reading data
~~~~~~~~~~~~

To read data from the Pozyx, a simple pattern is followed. This pattern can be used with almost all methods starting with the words 'get':

1. Initialize the appropriate container for your data read.
2. Pass this container along with the get functions.
3. Check the status to see if the operation was successful and thus the data trustworthy.

You can see the same pattern in action above when reading the UWB data.

.. TODO An overview of all data containers, their usage and their particularities can be found here:

.. TODO also mention that they all have human readable __str__ conversions

.. code-block:: python

    from pypozyx import PozyxSerial, get_first_pozyx_serial_port, POZYX_SUCCESS, SingleRegister, EulerAngles, Acceleration
    # initalize the Pozyx as above

    # initialize the data container
    who_am_i = SingleRegister()
    # get the data, passing along the container
    status = pozyx.getWhoAmI(who_am_i)

    # check the status to see if the read was successful. Handling failure is covered later.
    if status == POZYX_SUCCESS:
        # print the container. Note how a SingleRegister will print as a hex string by default.
        print(who_am_i) # will print '0x43'

    # and repeat
    # initialize the data container
    acceleration = Acceleration()
    # get the data, passing along the container
    pozyx.getAcceleration_mg(acceleration)

    # initialize the data container
    euler_angles = EulerAngles()
    # get the data, passing along the container
    pozyx.getEulerAngles_deg(euler_angles)


Writing data
~~~~~~~~~~~~

Writing data follows a similar pattern as reading, but making a container for the data is optional. This pattern can be used with all methods starting with the words 'set':

1. (Optional) Initialize the appropriate container with the right contents for your data write.
2. Pass this container or the right value along with the set functions.
3. Check the status to see if the operation was successful and thus the data written.

.. note::

   All set functions are tolerant for values that aren't per se a data object. An integer value or respectively fitting array with the relevant data as contained in the register will pass as well.

   .. code-block:: python

       # method 1: making a data object
       uwb_channel = SingleRegister(5)
       pozyx.setUWBChannel(uwb_channel)
       # method 2: or just using the channel number directly
       pozyx.setUWBChannel(5)

       # both have the same effect!

   The advantage of using the data object approach lies especially with more complex data, where your Python editor will give you more information on what content you're putting in, or where the object will convert data to the right form for you.

   .. code-block:: python

       # method 1: making a data object
       # this is much more readable
       uwb_settings = UWBSettings(channel=5, bitrate=1, prf=2, plen=0x08, gain_db=25.0)
       pozyx.setUWBChannel(uwb_channel)
       # method 2: using the register values directly
       # this isn't readable and also not writable (need to search in depth register documentation)
       pozyx.setUWBSettings([5, 0b10000001, 0x08, 50])

       # both still have the same effect, but note how bitrate and prf combine in a register value,
       # and gain is doubled when converted to its register contents.

Some typical write operations

.. code-block:: python

   from pypozyx import PozyxSerial, get_first_pozyx_serial_port, POZYX_SUCCESS, SingleRegister, PozyxConstants

   # initialize Pozyx as above

   pozyx.setPositionAlgorithm(PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY)

   new_id = NetworkId(0x1)
   pozyx.setNetworkId(new_id)

   pozyx.setPositioningFilter(PozyxConstant.FILTER_TYPE_MOVING_AVERAGE, 10)

Note that you seemingly need to know that the positioning filter has ``PozyxConstant.FILTER_TYPE_MOVING_AVERAGE`` as a possible type of filter. This is pretty low-level knowledge and may remain hidden when not knowing about, and so in a recent version we added a lot of helpers that do away with having to know the appropriate constants for certain operations.

.. code-block:: python

   # instead of pozyx.setPositionAlgorithm(PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY)
   pozyx.setPositionAlgorithmNormal()

   # instead of pozyx.setPositioningFilter(PozyxConstant.FILTER_TYPE_MOVING_AVERAGE, 10)
   pozyx.setPositioningFilterMovingAverage(10)

Performing functions
~~~~~~~~~~~~~~~~~~~~

Positioning, ranging, configuring the anchors for a tag to use... While the line is sometimes thin, these aren't per se writes or reads as they are functions on the Pozyx.

A Pozyx device function typically can take a container object for storing the function's return data, and a container object for the function parameters.

For example, when adding an anchor to a tag's device list, the anchor's ID and position are the function's parameters, but there is no return data. Thus, the function addDevice only needs a container object containing the anchor's properties.

In the library, function wrappers are written in such a way that when no parameters are required, they are hidden from the user, and the same goes for return data.

.. code-block:: python

   from pypozyx import ..., Coordinates, DeviceCoordinates

   # assume an anchor 0x6038 that we want to add to the device list and immediately save the device list after.
   anchor = DeviceCoordinates(0x6038), 0, Coordinates(5000, 5000, 0))
   pozyx.addDevice(anchor)
   pozyx.saveNetwork()

   # after, we can start positioning. Positioning takes its parameters from the configuration in the tag's
   # registers, and so we only need the coordinates.
   position = Coordinates()
   pozyx.doPositioning(position)

.. TODO find better example than positioning since that's a lie

Remote
~~~~~~

To interface with a remote device, every function has a remote_id optional parameter. Thus, every function you just saw can be performed on a remote device as well!

.. code-block:: python

    # let's assume there is another tag present with ID 0x6039
    remote_device_id = 0x6039

    # this will read the WHO_AM_I register of the remote tag
    who_am_i = SingleRegister()
    pozyx.getWhoAmI(who_am_i)
    print(who_am_i) # will print 0x43


Saving writable register data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Basically, every register you can write data to as a user can be saved in the device's flash memory. This means that when the device is powered on, its configuration will remain. Otherwise, the device will use its default values again.

.. TODO add default values for registers so that users know what to expect.

This is useful for multiple things:

* Saving the UWB settings so all your devices remain on the same UWB settings.
* Saving the anchors the tag uses for positioning. This means that after a reset, the tag can resume positioning immediately and doesn't need to be reconfigured!
* Saving positioning algorithm, dimension, filter... you'll never lose your favorite settings when the device shuts down.

There are various helpers in the library to help you save the settings you prefer, not requiring you to look up the relevant registers.

.. code-block:: python

    # Saves the positioning settings
    pozyx.savePositioningSettings()
    # Saves the device list used for positioning
    pozyx.saveNetwork()
    # Saves the device's UWB settings
    pozyx.saveUWBSettings()

Finding out the error
~~~~~~~~~~~~~~~~~~~~~

Pozyx functions typically return a status to indicate the success of the function. This is useful to indicate failure especially. When things go wrong, it's advised to read the error as well.

A code snippet shows how this is typically done

.. code-block:: python

    from pypozyx import PozyxSerial, get_first_pozyx_serial_port, POZYX_SUCCESS, SingleRegister

    # initialize Pozyx as above

    if pozyx.saveUWBSettings() != POZYX_SUCCESS:
        # this is one way which retrieves the error code
        error_code = SingleRegister()
        pozyx.getErrorCode(error_code)
        print('Pozyx error code: %s' % error_code)
        # the other method returns a descriptive string
        print(pozyx.getSystemError())
