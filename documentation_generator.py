import os
import subprocess

path = os.path.dirname(__file__)

if len(path) > 0:
    path += '/'

original_folder = path + 'pypozyx'
doxypy_folder = path + 'docs/doxypypozyx'

files = [
    'core.py',
    'lib.py',
    'pozyx_serial.py',
    'definitions/bitmasks.py',
    'definitions/constants.py',
    'definitions/registers.py',
    'structures/device.py',
    'structures/generic.py',
    'structures/sensor_data.py',
    'structures/byte_structure.py'
]

for file in files:
    command = ['doxypypy', '-a', '-c', original_folder + '/' + file, '>', doxypy_folder + '/' + file]
    command = ' '.join(command)
    subprocess.run(command, shell=True)

subprocess.run('doxygen doxygen_settings', shell=True)
