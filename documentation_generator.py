def clean_up_doxypypy_files(filename):
    lines_to_keep = []
    lines = []

    in_example = False
    with open(filename, 'r') as fp:
        lines += fp.readlines()
    for line in lines:
        if in_example is True:
            if '#' not in line:
                in_example = False
            else:
                continue
        if 'Examples' in line:
            in_example = True
            continue
        parts = line.split("# @return")
        if len(parts) == 1:
            lines_to_keep.append(line)
        else:
            if len(parts[1].strip()) == 0:
                continue
            else:
                lines_to_keep.append(line.replace('@return', '@retval').replace('POZYX_', '#POZYX_'))


    with open(filename, 'w') as fp:
        fp.writelines(lines_to_keep)



if __name__ == '__main__':
    import os
    import subprocess
    import shutil

    path = os.path.dirname(__file__)

    if len(path) > 0:
        path += '/'

    original_folder = path + 'pypozyx'
    doxypy_folder = path + 'docs/doxypypozyx'

    def run_doxygen(files):
        os.mkdir(doxypy_folder)
        os.mkdir(doxypy_folder + '/definitions')
        os.mkdir(doxypy_folder + '/structures')

        for file in files:
            command = ['doxypypy', '-a', '-c', original_folder + '/' + file, '>', doxypy_folder + '/' + file]
            command = ' '.join(command)
            subprocess.run(command, shell=True)

        for file in files:
            clean_up_doxypypy_files(doxypy_folder + '/' + file)

        subprocess.run('doxygen doxygen_settings', shell=True)



    # generate groups
    files_first_run = [
        'core.py',
        'lib.py',
        'pozyx_serial.py'
        # 'definitions/bitmasks.py',
        # 'definitions/constants.py',
        # 'definitions/registers.py',
        # 'structures/device.py',
        # 'structures/generic.py',
        # 'structures/sensor_data.py',
        # 'structures/byte_structure.py'
    ]

    # generate relevant classes in index
    files_second_run = [
        'structures/device.py',
        'structures/generic.py',
        'structures/sensor_data.py'
    ]

    run_doxygen(files_first_run)
    shutil.rmtree(doxypy_folder)
    run_doxygen(files_second_run)
