import sys, os, ctypes, datetime
import numpy as np
from scipy import signal

enable_matplotlib_preview = False # Requires matplotlib
enable_plotly_preview = True # Requires plotly
enable_map_view = True # Requires plotly

if enable_matplotlib_preview:
    import matplotlib.pyplot as plt
if enable_plotly_preview or enable_map_view:
    import pandas as pd
    import plotly.express as px
    import plotly.subplots as sp

delay_pz_analog = 300e-6 # 300 µs
delay_mems = 12.25e-3 # 12.25 ms
offset_pz = 0.0
version_support = 1
img_dir = 'vera2csv_img'
img_scale = 2.0

# Reads acceleration file, returns (a_header, a_data_points)
def a_parse(a_path, n_skip):
    with open(a_path, 'rb') as f:
        a_data = f.read() # Read binary acceleration data
    if len(a_data) == 0:
        return None, None

    # Definition of a_data_header_t
    a_version = a_data[0]
    if a_version == 1:
        class A_DataHeader(ctypes.Structure):
            _fields_ = (
                ('version', ctypes.c_uint8),
                ('boot_duration', ctypes.c_uint32),
                ('a_buffer_len', ctypes.c_uint32),
                ('a_sampling_rate', ctypes.c_uint32),
                ('piezo_count_max', ctypes.c_uint8),
                ('piezo_count', ctypes.c_uint8),
                ('oversampling_ratio', ctypes.c_uint8),
                ('fir_taps_len', ctypes.c_uint32),
            )
    else:
        print(f'! ERROR: Acceleration data version {a_version} is not supported. This script version supports min. 1 max. {version_support}.')
        exit()
    a_hd_t = A_DataHeader # Type to use for header parsing
    a_header = a_hd_t.from_buffer_copy(a_data[:ctypes.sizeof(a_hd_t)]) # Parse from byte array
    a_data = a_data[ctypes.sizeof(a_hd_t):] # Remove header from byte array

    # Definition of a_data_point_t
    class A_DataPoint(ctypes.Structure):
        _fields_ = (
            ('complete', ctypes.c_uint8),
            ('timestamp', ctypes.c_uint32),
            ('temp_mems1', ctypes.c_uint16),
            ('xyz_mems1', ctypes.c_int32 * 3),
            ('a_piezo', ctypes.c_int16 * a_header.piezo_count_max),
        )
    a_data_points = [] # Define list for parsed data (will be filled with instances of A_DataPoint)
    a_dp_t = A_DataPoint # Type to use for parsing
    for i in range(0, len(a_data), ctypes.sizeof(a_dp_t) * (n_skip + 1)): # Step through binary data, step size is sizeof(a_data_point_t)
        dp_slice = a_data[i:i + ctypes.sizeof(a_dp_t)] # Region of binary data for current A_DataPoint
        if len(dp_slice) >= ctypes.sizeof(a_dp_t):
            a_data_points.append(a_dp_t.from_buffer_copy(dp_slice)) # Parse and add A_DataPoint from binary data
    return a_header, a_data_points

# Reads position file, returns (p_header, p_data_points)
def p_parse(p_path, n_skip):
    with open(p_path, 'rb') as f:
        p_data = f.read()
    if len(p_data) == 0:
        return None, None

    p_version = p_data[0]
    if p_version == 1:
        class P_DataHeader(ctypes.Structure):
            _fields_ = (
                ('version', ctypes.c_uint8),
                ('boot_duration', ctypes.c_uint32),
                ('p_buffer_len', ctypes.c_uint32),
                ('p_sampling_rate', ctypes.c_uint32),
                ('year', ctypes.c_uint16),
                ('month', ctypes.c_uint8),
                ('day', ctypes.c_uint8),
            )
    else:
        print(f'! ERROR: Position data version {p_version} is not supported. This script version supports min. 1 max. {version_support}.')
        exit()
    p_hd_t = P_DataHeader
    p_header = p_hd_t.from_buffer_copy(p_data[:ctypes.sizeof(p_hd_t)])
    p_data = p_data[ctypes.sizeof(p_hd_t):]

    class P_DataPoint(ctypes.Structure):
        _fields_ = (
            ('complete', ctypes.c_uint8),
            ('timestamp', ctypes.c_uint32),
            ('gnss_hour', ctypes.c_uint8),
            ('gnss_minute', ctypes.c_uint8),
            ('gnss_second', ctypes.c_float),
            ('lat', ctypes.c_float),
            ('lon', ctypes.c_float),
            ('speed', ctypes.c_float),
            ('altitude', ctypes.c_float),
        )
    p_data_points = []
    p_dp_t = P_DataPoint
    for i in range(0, len(p_data), ctypes.sizeof(p_dp_t) * (n_skip + 1)):
        dp_slice = p_data[i:i + ctypes.sizeof(p_dp_t)]
        if len(dp_slice) >= ctypes.sizeof(p_dp_t):
            p_data_points.append(p_dp_t.from_buffer_copy(dp_slice))
    return p_header, p_data_points

# Returns datetime object in local timezone based on GNSS date from position data point
def p_dp_datetime(dp):
    utc_date = [p_header.year, p_header.month, p_header.day]
    utc_time = [dp.gnss_hour, dp.gnss_minute, int(dp.gnss_second), int((dp.gnss_second % 1) * 1000)]
    utc_datetime = datetime.datetime(*utc_date, *utc_time, tzinfo=datetime.timezone.utc)
    return utc_datetime.astimezone()

# Checks if array exceeds maximum, returns True if a <= mx
def check_max(a, mx):
    if np.max(a) > mx:
        print(f'{np.max(a)} > {mx}')
        return False
    return True

# Checks if array exceeds minimum or maximum, returns True if mn <= a <= mx
def check_min_max(a, mn, mx):
    if np.min(a) < mn:
        print(f'! Value too small: {np.min(a)} < {mn}')
        return False
    if np.max(a) > mx:
        print(f'! Value too large: {np.max(a)} > {mx}')
        return False
    return True

if '-h' in sys.argv or '--help' in sys.argv:
    print('Usage: python vera2csv.py (options) [path like "./24_08_01-1" or "./24_08_01-1/a_0.bin"]')
    print('\tIf no path argument is given, the script tries finding files in the current working directory')
    print('Options:')
    print('\t-t0  / --time0 (sec)        | Sets time (in seconds) to start saving data')
    print('\t-t1  / --time1 (sec)        | Sets time (in seconds) to stop saving data')
    print('\t-d   / --duration (sec)     | Sets duration (in seconds) of data, starting from -t0')
    print('\t-ni  / --no-interpolate     | Disable linear interpolation of coordinates')
    print('\t-sk  / --skip (N)           | Skips N data points (0: every data point)')
    print('\t-fir / --firfilter          | Filters data with 4 Hz lowpass')
    print('\t-tf  / --timeformat (0-2)   | Set timestamp format in .csv (0: ISO format, 1: POSIX timestamp, 2: hour,minute,second)')
    print('\t-p   / --preview            | Shows interactive preview of acceleration data')
    print('\t-pf  / --previewfft (axis)  | Shows interactive preview including FFT of specified axis (0-2: MEMS [X-Z], 3-7: Piezo [0-4])')
    print('\t-m   / --map                | Shows interactive map preview of position data')
    print('\t-sp  / --saveplot           | Saves selected plots (see above options) as .png and .pdf')
    print('\t-s   / --save (path)        | Saves data as .csv file')
    print('\t-ow  / --overwrite          | Automatically overwrite .csv file if it exists')
    print('If neither -p/-pf/-m nor -s are selected, the script only checks data for integrity')
    print('IMPORTANT: --skip 32 is highly recommended for PREVIEW of measurements longer than 10 minutes (128 for multiple hours), for full export do not use --skip')
    exit()

# Parse arguments
arg_t0 = 0
arg_t1 = None
arg_d = None
arg_ni = False
arg_skip = 0
arg_tf = 0
arg_preview = False
arg_fftaxis = None
arg_map_view = False
arg_sp = False
arg_save = None
arg_ow = False
arg_path = './'
argv_i = 1

arg_fir = False
while argv_i < len(sys.argv):
    a = sys.argv[argv_i]
    if a in ['-t0', '--time0']:
        arg_t0 = float(sys.argv[argv_i + 1])
        argv_i += 1
    elif a in ['-t1', '--time1']:
        arg_t1 = float(sys.argv[argv_i + 1])
        argv_i += 1
    elif a in ['-d', '--duration']:
        arg_d = float(sys.argv[argv_i + 1])
        argv_i += 1
    elif a in ['-ni', '--no-interpolate']:
        arg_ni = True
    elif a in ['-sk', '--skip']:
        arg_skip = int(sys.argv[argv_i + 1])
        argv_i += 1
    elif a in ['-tf', '--timeformat']:
        arg_tf = int(sys.argv[argv_i + 1])
        argv_i += 1
    elif a in ['-p', '--preview']:
        arg_preview = True
    elif a in ['-pf', '--previewfft']:
        arg_fftaxis = int(sys.argv[argv_i + 1])
        argv_i += 1
    elif a in ['-m', '--map']:
        arg_map_view = True
    elif a in ['-sp', '--saveplot']:
        arg_sp = True
    elif a in ['-s', '--save']:
        arg_save = sys.argv[argv_i + 1]
        argv_i += 1
    elif a in ['-ow', '--overwrite']:
        arg_ow = True
    elif a in ['-fir', '--firfilter']:
        arg_fir = True
    else:
        if a.startswith('-'):
            print(f"! WARNING: Possibly unknown option {a}, except if it's part of the path")
        arg_path = a
    argv_i += 1

if arg_sp and not os.path.isdir(img_dir):
    os.mkdir(img_dir)

# Find files
a_file_paths = []
p_file_paths = []
if os.path.isdir(arg_path):
    file_index = 0
    dir_path = arg_path
elif os.path.isfile(arg_path):
    file_index = int(os.path.basename(arg_path).split('.')[0].split('_')[-1])
    dir_path = os.path.dirname(arg_path)
else:
    print(f'! ERROR: Cannot read path "{arg_path}"')
    exit()
print(f'Scanning directory "{dir_path}"')
while True:
    a_path = os.path.join(dir_path, f'a_{file_index}.bin')
    p_path = os.path.join(dir_path, f'p_{file_index}.bin')
    if os.path.isfile(a_path) and os.path.isfile(p_path):
        a_file_paths.append(a_path)
        p_file_paths.append(p_path)
    elif file_index > 0:
        break
    file_index += 1
pp = None
if os.path.isfile(os.path.join(dir_path, 'preprocess.py')):
    sys.path.append(dir_path)
    import preprocess # type: ignore
    pp = preprocess.preprocess

# Check found paths
if len(a_file_paths) == 0 and len(p_file_paths) == 0:
    print('! ERROR: No input files found')
    exit()
if len(a_file_paths) == 0:
    print('! WARNING: No acceleration files (a_X.bin)')
if len(p_file_paths) == 0:
    print('! WARNING: No position files (p_X.bin)')

# Parse data points
file_count = len(a_file_paths) + len(p_file_paths)
print(f'Reading and parsing {file_count} file(s)...')
print('0.0 %')
a_data_points = []
p_data_points = []
# Parse the number in the filename, since sorting by string would result in ['1', '10', '11', '2', '3', ...]
def get_file_num(f):
    basename = os.path.splitext(os.path.basename(f))[0]
    return int(basename[2:])
a_file_paths = sorted(a_file_paths, key=get_file_num)
p_file_paths = sorted(p_file_paths, key=get_file_num)
max_len = max(len(a_file_paths), len(p_file_paths))
stop_at_next = False

for i in range(max_len):
    if stop_at_next:
        break
    if i < len(a_file_paths):
        a_header, a_dp = a_parse(a_file_paths[i], arg_skip)
        if a_header is None or a_dp is None or len(a_dp) == 0:
            continue
        if arg_t1 is not None:
            if a_dp[-1].timestamp / (a_header.a_sampling_rate / (arg_skip + 1)) > arg_t1:
                stop_at_next = True
        elif arg_d is not None:
            if a_dp[-1].timestamp / (a_header.a_sampling_rate / (arg_skip + 1)) > arg_t0 + arg_d:
                stop_at_next = True
        if a_dp[-1].timestamp / (a_header.a_sampling_rate / (arg_skip + 1)) >= arg_t0:
            a_data_points.extend(a_dp)
    if i < len(p_file_paths):
        p_header, p_dp = p_parse(p_file_paths[i], arg_skip)
        if p_header is None or p_dp is None or len(p_dp) == 0:
            continue
        if arg_t1 is not None:
            if p_dp[-1].timestamp / (a_header.a_sampling_rate / (arg_skip + 1)) > arg_t1:
                stop_at_next = True
        elif arg_d is not None:
            if p_dp[-1].timestamp / (a_header.a_sampling_rate / (arg_skip + 1)) > arg_t0 + arg_d:
                stop_at_next = True
        if p_dp[-1].timestamp / (a_header.a_sampling_rate / (arg_skip + 1)) >= arg_t0:
            p_data_points.extend(p_dp)
    print(round(100 * (i + 1) / max_len, 2), '%')
print("a_data_header:")
for key, f_type in type(a_header)._fields_:
    print(f" {key}: {getattr(a_header, key)}")
print("p_data_header:")
for key, f_type in type(p_header)._fields_:
    print(f" {key}: {getattr(p_header, key)}")

print(f'Loaded "{arg_path}"')

# Check date
if p_header.year < 1800:
    if p_header.month < 1 or p_header.month > 12 or p_header.day < 1 or p_header.day > 31:
        p_header.year = datetime.datetime.now().year
        p_header.month = datetime.datetime.now().month
        p_header.day = datetime.datetime.now().day
        print(f'! WARNING: Date is invalid, assuming today ({str(p_header.year).zfill(4)}-{str(p_header.month).zfill(2)}-{str(p_header.day).zfill(2)})')
    else:
        p_header.year = datetime.datetime.now().year
        print(f'! WARNING: Year is invalid, assuming {str(p_header.year).zfill(4)}-{str(p_header.month).zfill(2)}-{str(p_header.day).zfill(2)}')

# Remove first sample after boot (invalid)
a_data_points = a_data_points[1:]
p_data_points = p_data_points[1:]

# Sort by GNSS time
def get_p_dp_time(dp):
    return 3600.0 * dp.gnss_hour + 60.0 * dp.gnss_minute + dp.gnss_second
gnss_times = np.array([get_p_dp_time(dp) for dp in p_data_points])
gnss_times_valid = np.array([dp.complete & (1 << 1) != 0 for dp in p_data_points])
for i in range(len(gnss_times) - 1):
    if gnss_times_valid[i] and gnss_times_valid[i + 1]:
        if gnss_times[i + 1] < gnss_times[i] and gnss_times[i] > 1 and gnss_times[i + 1] > 1:
            true_i = np.argmin(np.abs(gnss_times + 1.0 / p_header.p_sampling_rate - gnss_times[i + 1]))
            unchanged = p_data_points[true_i + 1:i + 1]
            p_data_points[true_i + 1] = p_data_points[i + 1]
            p_data_points[true_i + 2:i + 2] = unchanged

# Get time data
def get_x_data(data_points):
    x = []
    for dp in data_points:
        x.append(dp.timestamp / a_header.a_sampling_rate)
        if arg_t1 is not None:
            if x[-1] > arg_t1:
                break
        elif arg_d is not None:
            if x[-1] > arg_t0 + arg_d:
                break
    return np.array(x)

full_data_x = get_x_data(a_data_points)
full_data_x_p = get_x_data(p_data_points)
full_data_x -= full_data_x[0]
full_data_x_p -= full_data_x[0]

# Get timespan
t_start = arg_t0
t_end = full_data_x[-1]
if arg_t1 is not None:
    t_end = arg_t1
elif arg_d is not None:
    t_end = arg_t0 + arg_d
i_start = np.argmin(np.abs(full_data_x - t_start))
i_end = np.argmin(np.abs(full_data_x - t_end))
if len(full_data_x_p) > 0:
    i_start_p = np.argmin(np.abs(full_data_x_p - t_start))
    i_end_p = np.argmin(np.abs(full_data_x_p - t_end))

# Calculate group delay
delay_pz_digital = (a_header.fir_taps_len - 1) / 2.0 / a_header.a_sampling_rate
delay_piezo = delay_pz_analog + delay_pz_digital
delay_piezo_i = int(delay_piezo * a_header.a_sampling_rate / (arg_skip + 1))
delay_mems_i = int(delay_mems * a_header.a_sampling_rate / (arg_skip + 1))

if arg_t0 > 0 or arg_t1 is not None or arg_d is not None:
    duration = t_end - t_start
    print(f"Cropped duration: {str(int(duration / 3600)).zfill(2)}:{str(int(duration / 60) % 60).zfill(2)}:{round(duration % 60, 3)} ({round(t_start, 3)} s - {round(t_end, 3)} s)")
else:
    print(f"Total duration: {str(int(full_data_x[-1] / 3600)).zfill(2)}:{str(int(full_data_x[-1] / 60) % 60).zfill(2)}:{round(full_data_x[-1] % 60, 3)}")

# Crop x axis
full_data_x = full_data_x[i_start:i_end]
if len(full_data_x) == 0:
    if len(a_data_points) > 0:
        print("! ERROR: Cropped out all data (check options -t0 -t1 -d and actual duration of given data)")
    else:
        print("! ERROR: No data")
    exit()
a_data_points = a_data_points[i_start:i_end]
if len(full_data_x_p) > 0:
    full_data_x_p = full_data_x_p[i_start_p:i_end_p]
    p_data_points = p_data_points[i_start_p:i_end_p]

# Convert and scale acceleration data
full_data_y = []
# Scale to -1 <= y <= 1
for a in range(3):
    full_data_y.append(np.array([dp.xyz_mems1[a] for dp in a_data_points[delay_mems_i:]]) / 524287.0)
for i in range(a_header.piezo_count):
    full_data_y.append(2 * np.array([dp.a_piezo[i] for dp in a_data_points[delay_piezo_i:]]) / 8190.0)
# Extend missing due to delay
if len(full_data_y) > 3:
    for a in range(3):
        if len(full_data_y[a]) < len(full_data_y[3]):
            full_data_y[a] = np.append(full_data_y[a], [0] * (len(full_data_y[3]) - len(full_data_y[a])))
    for i in range(a_header.piezo_count):
        if len(full_data_y[3 + i]) < len(full_data_y[0]):
            full_data_y[3 + i] = np.append(full_data_y[3 + i] + offset_pz, [0] * (len(full_data_y[0]) - len(full_data_y[3 + i])))
full_data_x = full_data_x[:len(full_data_y[0])]
full_data_y = np.array(full_data_y).clip(-1, 1)
# Preprocess
if pp is not None:
    try:
        print('Running preprocess.py')
        full_data_y = pp(full_data_y)
    except Exception as e:
        print(f'! WARNING: Preprocess script failed: {e}')

# Check data
piezos_missing = False
mems_missing = False
if len(a_data_points) == 0:
    print('! ERROR: No acceleration data')
    exit()
elif arg_skip != 0:
    print('! WARNING: Option --skip in use, data integrity will not be checked')
else:
    cplt = np.array([dp.complete for dp in a_data_points])
    if not check_min_max(cplt, 7, 7):
        print('! WARNING: Incomplete acceleration data points')
        if np.any(cplt & (1 << 0) == 0):
            print('! WARNING: Some acceleration data points are missing timestamps. This should not have happened.')
        if np.all(cplt & (1 << 1) == 0):
            mems_missing = True
            print('! WARNING: All MEMS data missing')
        if np.all(cplt & (1 << 2) == 0):
            piezos_missing = True
            print('! WARNING: All piezo data missing')
    if not check_min_max(np.diff(full_data_x), 0.9 / a_header.a_sampling_rate, 1.1 / a_header.a_sampling_rate):
        print('! WARNING: Missing acceleration timestamps (bad!)')
    for a in range(3):
        if not check_min_max([dp.xyz_mems1[a] for dp in a_data_points], -524287, 524287):
            print(f'! WARNING: MEMS {["X", "Y", "Z"][a]} data out of range')
    for i in range(a_header.piezo_count):
        if not check_min_max([dp.a_piezo[i] for dp in a_data_points], -8190, 8190):
            print(f'! WARNING: Piezo [{i}] data out of range')

gnss_times_missing = False
pos_missing = False
speed_missing = False
altitude_missing = False
if len(p_data_points) == 0:
    print('! WARNING: No position data')
elif arg_skip != 0:
    pass
else:
    cplt = np.array([dp.complete for dp in p_data_points])
    if not check_min_max(cplt, 3, 31):
        print('! WARNING: Incomplete position data points')
        if np.any(cplt & (1 << 0) == 0):
            print('! WARNING: Some position data points are missing timestamps. This should not have happened.')
        if np.all(cplt & (1 << 1) == 0):
            gnss_times_missing = True
            print('! WARNING: All GNSS times missing')
        if np.all(cplt & (1 << 2) == 0):
            pos_missing = True
            print('! WARNING: All coordinates missing')
        if np.all(cplt & (1 << 3) == 0):
            speed_missing = True
            print('! WARNING: All speed data missing')
        if np.all(cplt & (1 << 4) == 0):
            altitude_missing = True
            print('! WARNING: All altitude data missing')
    if not check_max(np.diff(full_data_x_p), 30):
        print('! WARNING: Missing position timestamps')
    if np.all(np.abs(np.array([dp.lat for dp in p_data_points])) <= 1) or np.all(np.abs(np.array([dp.lon for dp in p_data_points])) <= 1):
        print('! WARNING: No position data (invalid)')

# Export .csv
if arg_save is not None:
    csv_line_sep = '\n'

    do_save = 1
    if os.path.isfile(arg_save) and not arg_ow:
        confirm = input(f'! WARNING: File "{arg_save}" exists. Do you want to overwrite it? (y/n) ')
        do_save = confirm.lower() in ['y', 'yes']
    if do_save:
        with open(arg_save, 'w') as f:
            csv_header = ['time']
            if not mems_missing:
                csv_header.extend(['temp_mems1'])
            for i in range(len(full_data_y)):
                if i < 3 and mems_missing:
                    continue
                if i >= 3 and piezos_missing:
                    continue
                csv_header.append(f"{['x', 'y', 'z'][i]}_mems1" if i < 3 else f"a_piezo{i - 2}")
            if len(full_data_x_p) > 0:
                if not gnss_times_missing:
                    if arg_tf == 0:
                       csv_header.extend(['gnss_time_iso'])
                    elif arg_tf == 1:
                       csv_header.extend(['gnss_timestamp'])
                    elif arg_tf == 2:
                       csv_header.extend(['gnss_hour', 'gnss_minute', 'gnss_second'])
                    else:
                        print(f'! WARNING: Unknown time format {arg_tf}')
                if not pos_missing:
                    csv_header.extend(['lat', 'lon'])
                if not speed_missing:
                    csv_header.extend(['speed'])
                if not altitude_missing:
                    csv_header.extend(['altitude'])
            f.write(','.join(csv_header) + csv_line_sep)

            for a_dp_i in range(len(full_data_x)):
                t = full_data_x[a_dp_i]
                if len(full_data_x_p) > 0:
                    p_dp_i = np.argmin(np.abs(full_data_x_p - t))
                    p_dp = p_data_points[p_dp_i]

                csv = [round(t, 6)]
                if not mems_missing:
                    csv.extend([a_data_points[a_dp_i].temp_mems1])
                for i in range(len(full_data_y)):
                    if i < 3 and mems_missing:
                        continue
                    if i >= 3 and piezos_missing:
                        continue
                    csv.append(round(full_data_y[i][a_dp_i], 6))
                if len(full_data_x_p) > 0:
                    if arg_ni or p_dp_i >= len(p_data_points) - 1:
                        if not gnss_times_missing:
                            dp_time = p_dp_datetime(p_dp)
                            if arg_tf == 0:
                                csv.extend([dp_time.time().isoformat()])
                            elif arg_tf == 1:
                                csv.extend([dp_time.timestamp()])
                            elif arg_tf == 2:
                                csv.extend([dp_time.hour, dp_time.minute, round(dp_time.second + (dp_time.microsecond * 1e-6), 3)])
                        if not pos_missing:
                            csv.extend([round(p_dp.lat, 8), round(p_dp.lon, 8)])
                        if not speed_missing:
                            csv.extend([round(p_dp.speed, 3)])
                        if not altitude_missing:
                            csv.extend([round(p_dp.altitude, 3)])
                    else:
                        # TODO: interpolate p_dp and p_data_points[p_dp_i + 1]
                        # TODO: better: save last valid (>0) time/position/speed/altitude, lookahead for next valid, interpolate individually, because not every position dp has valid data
                        # save index of previous and next valid
                        # if a.timestamp > p[next].timestamp: previous=next, find next
                        pass
                f.write(','.join([str(v) for v in csv]) + csv_line_sep)
        print(f'File "{arg_save}" written')


# TODO
if False:
    import matplotlib.pyplot as plt
    x1 = full_data_y[0] * 15
    x2 = full_data_y[3]
    f, Pxx = signal.csd(x1, x1, nperseg=1024, fs=a_header.a_sampling_rate)
    f, Pyy = signal.csd(x2, x2, nperseg=1024, fs=a_header.a_sampling_rate)
    f, Pxy = signal.csd(x1, x2, nperseg=1024, fs=a_header.a_sampling_rate)
    f, Pyx = signal.csd(x2, x1, nperseg=1024, fs=a_header.a_sampling_rate)
    dB = lambda y: 20 * np.log10(np.abs(y))
    plt.semilogx(f, dB(Pxx), label = '$P_{xx}$')
    plt.semilogx(f, dB(Pyy), label = '$P_{yy}$')
    # plt.semilogx(f, dB(Pyx/Pxx), label = '$H_{1}(f)$')
    # plt.semilogx(f, dB(Pyy/Pxy), label = '$H_{2}(f)$')
    plt.semilogx(f, (dB(Pyx/Pxx) + dB(Pyy/Pxy)) / 2, label = '$H_{1-2}(f)$')
    plt.legend()
    plt.show()

if arg_fir:
    fs = a_header.a_sampling_rate / (arg_skip + 1)
    f = 10
    numtaps = 1024
    h = signal.firwin(numtaps, f, fs = fs)
    fir_delay_i = int((numtaps - 1) / 2)
    plot_x = full_data_x[:-fir_delay_i]
    plot_y = []
    for i in range(len(full_data_y)):
        plot_y.append(signal.lfilter(h, [1.0], full_data_y[i])[fir_delay_i:])
else:
    plot_x = full_data_x
    plot_y = full_data_y

show_plot = arg_preview or arg_fftaxis is not None
show_fft = arg_fftaxis is not None
show_coords = arg_map_view and not enable_map_view
if show_plot or show_fft or show_coords:
    if not enable_matplotlib_preview and not enable_plotly_preview:
        print('! ERROR: Preview disabled (check settings at the beginning of vera2csv.py)')
    if enable_plotly_preview:
        fig = sp.make_subplots(rows = 2 if show_fft else 1, cols = 1, shared_xaxes = True)

        # Plotly graphs
        mems_axes = {}
        if not mems_missing:
            mems_axes = {
                **{f"MEMS {['X', 'Y', 'Z'][i]}": plot_y[i] for i in range(len(full_data_y))[:3]},
            }
        piezo_axes = {}
        if not piezos_missing:
            piezo_axes = {
                **{f"Piezo [{i - 3}]": plot_y[i] for i in range(len(full_data_y))[3:]},
            }
        axes = {**mems_axes, **piezo_axes}
        # axes = {list(axes.keys())[arg_fftaxis]: list(axes.values())[arg_fftaxis]}
        df = pd.DataFrame({'t': plot_x, **axes})
        fig.add_traces(px.line(df, x = 't', y = df.columns, hover_data = {'t': False}).data, rows = 1, cols = 1)

        if show_fft:
            # Plotly spectrogram
            if arg_fftaxis < 0 or arg_fftaxis >= len(full_data_y):
                print('! WARNING: Invalid FFT axis specified (value after -pf should be between 0 and 5), using first axis (0)')
                arg_fftaxis = 0

            win = signal.windows.blackman(1024)
            SFT = signal.ShortTimeFFT(win, hop = 64, fs = a_header.a_sampling_rate / (arg_skip + 1), scale_to = 'magnitude')
            max_scale = 1
            if np.max(np.abs(full_data_y[arg_fftaxis])) > 1e-9:
                max_scale = np.max(np.abs(full_data_y[arg_fftaxis]))
            Sx2 = SFT.spectrogram(20 * full_data_y[arg_fftaxis] / max_scale)
            Sx2_dB = 20 * np.log10(Sx2.clip(1e-5, 1))

            t0, t1, f0, f1 = SFT.extent(len(full_data_y[arg_fftaxis]))
            fig.add_traces(px.imshow(Sx2_dB,
                                     aspect = 'auto', origin = 'lower',
                                     x = np.linspace(t0, t1, Sx2.shape[1]) + t_start,
                                     y = np.linspace(f0, f1, Sx2.shape[0])).data,
                                     rows = 2, cols = 1)

        fig.update_traces(hovertemplate = '%{y:.6f}')
        fig.update_layout(
            font_size = 14,
            xaxis = {'ticksuffix': ' s'},
            xaxis2 = {'title': '<i>t</i>', 'ticksuffix': ' s'},
            yaxis = {'title': '<i>a</i>'},
            yaxis2 = {'title': '<i>f</i> / Hz', 'range': [0, 300]},
            hovermode = 'x unified',
            colorscale = {'sequentialminus': 'aggrnyl'},
            coloraxis_colorbar = {
                'title': '<i>S<sub>XX</sub>(f)</i> / dB',
                'len': 0.5, 'lenmode': 'fraction',
                'y': 0, 'yanchor': 'bottom',
            }
        )
        if arg_sp:
            print('Writing graph.png...')
            fig.write_image('vera2csv_img/graph.png', scale = img_scale)
            print('Writing graph.pdf...')
            fig.write_image('vera2csv_img/graph.pdf', scale = img_scale)
        else:
            fig.show()
    if enable_matplotlib_preview:
        fig = plt.figure()

        ax_p = None
        ax_f = None
        ax_c = None
        if not show_plot:
            if not show_fft and show_coords:
                # 2x1
                axs = fig.subplots(2, 1, sharex = 'all')
                ax_c = axs
        else:
            if not show_fft and not show_coords:
                # 1x1
                axs = fig.subplots()
                ax_p = axs
            if not show_fft and show_coords:
                # 2x2 (plot stretched over rows)
                ax_p = fig.add_subplot(2, 2, (1, 3))
                ax_c = [fig.add_subplot(2, 2, 2, sharex = ax_p), fig.add_subplot(2, 2, 4, sharex = ax_p)]
            if show_fft and not show_coords:
                # 2x1
                axs = fig.subplots(2, 1, sharex = 'all')
                ax_p = axs[0]
                ax_f = axs[1]
            if show_fft and show_coords:
                # 2x2
                axs = fig.subplots(2, 2, sharex = 'all')
                ax_p = axs[0][0]
                ax_f = axs[1][0]
                ax_c = [axs[0][1], axs[1][1]]

        if show_plot and ax_p is not None:
            # matplotlib graphs
            for i in range(len(full_data_y)):
                if i < 3 and mems_missing:
                    continue
                if i >= 3 and piezos_missing:
                    continue
                label = f"MEMS {['X', 'Y', 'Z'][i]}" if i < 3 else f"Piezo [{i - 3}]"
                ax_p.plot(full_data_x,
                        full_data_y[i],
                        label = label,
                        linewidth=1)
            ax_p.legend(loc=2)
            ax_p.set(xlabel='$t$ / s', ylabel='$a$')

        if show_fft and ax_f is not None:
            # matplotlib spectrogram
            ax_f.specgram(full_data_y[arg_fftaxis],
                          Fs = a_header.a_sampling_rate / (arg_skip + 1),
                          vmin = -100, vmax = 0,
                          xextent = (t_start, t_end))
            ax_f.set(xlabel='$t$ / s', ylabel='$f$ / Hz')

        if show_coords and ax_c is not None:
            full_data_lat = np.array([dp.lat for dp in p_data_points])
            full_data_lon = np.array([dp.lon for dp in p_data_points])
            ax_c[0].plot(full_data_x_p[np.abs(full_data_lat) > 1],
                         full_data_lat[np.abs(full_data_lat) > 1])
            ax_c[0].set(ylabel='Latitude')
            ax_c[1].plot(full_data_x_p[np.abs(full_data_lon) > 1],
                         full_data_lon[np.abs(full_data_lon) > 1])
            ax_c[1].set(xlabel='$t$ / s', ylabel='Longitude')

        if arg_sp:
            print("! WARNING: Option -sp/--saveplot only works with plotly, not matplotlib")
        plt.show()

if arg_map_view and len(p_data_points) > 0:
    if not enable_map_view:
        print('! WARNING: Map preview disabled (check settings at the beginning of vera2csv.py), showing coordintes as graphs')
    elif pos_missing:
        print('! WARNING: Can\'t show map view, since position data is missing')
    else:
        # Show plotly map
        df = {}
        df['name'] = []
        df['Messung'] = []
        df['Lat'] = []
        df['Lon'] = []
        df_extra = {}
        if not gnss_times_missing:
            df_extra['Time'] = []
        if not speed_missing:
            df_extra['Speed'] = []
        if not altitude_missing:
           df_extra['Altitude'] = []
        for dp_i in range(len(p_data_points)):
            dp = p_data_points[dp_i]
            if np.abs(dp.lat) > 1 and np.abs(dp.lon) > 1:
                df['name'].append(str(round(full_data_x_p[dp_i], 3)) + ' s')
                df['Messung'].append('1')
                df['Lat'].append(dp.lat)
                df['Lon'].append(dp.lon)
                if not gnss_times_missing:
                    df_extra['Time'].append(p_dp_datetime(dp).time().isoformat('milliseconds'))
                if not speed_missing:
                    df_extra['Speed'].append(str(round(dp.speed, 2)) + ' km/h')
                if not altitude_missing:
                    df_extra['Altitude'].append(str(round(dp.altitude, 2)) + ' m')
        cmap = {
            '1': '#FF0000',
        }
        fig = px.line_mapbox(pd.DataFrame({**df, **df_extra}),
                             title = f'VeRa Messung "{arg_path}"',
                             mapbox_style = 'open-street-map',
                             zoom = 7,
                             color_discrete_map = cmap, color = 'Messung',
                             lat = 'Lat', lon = 'Lon',
                             hover_name = 'name', hover_data = {'Messung': False, **{k: True for k in df_extra.keys()}})
        if arg_sp:
            print('Writing map.png...')
            fig.write_image('vera2csv_img/map.png', scale = img_scale)
            print('Writing map.pdf...')
            fig.write_image('vera2csv_img/map.pdf', scale = img_scale)
        else:
            fig.show()
