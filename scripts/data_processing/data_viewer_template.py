# HKJF
import tkinter as tk
from tkinter import filedialog
import numpy
import matplotlib.pyplot as plt
import csv

def select_and_load_data(file_title="Select CSV File"):
    root = tk.Tk()
    root.withdraw()

    filepath = filedialog.askopenfilename(
        initialdir=".",
        title=file_title,
        filetypes=(("CSV Files", "*.csv"), ("All Files", "*.*"))
    )

    if not filepath:
        print(f"No file selected for '{file_title}'")
        return None, None

    with open(filepath, 'r', newline='') as file:
        reader = csv.reader(file)
        headers = ["ts","Fx","Fy","Fz","Tx","Ty","Tz","x","y","z"]
        headers = {headers: i for i, headers in enumerate(headers)}
        data = numpy.array(list(reader)).astype(float)
        data[:,0] =  data[:,0] -  data[0,0]


    print(f"Current file name is {filepath}; \n"
          f"Headers for this file are: \n {headers}")
    return headers, data


def align_timestamp(LF_data, LF_ts, HF_ts):
    """Aligns low-frequency data to high-frequency timestamps using linear interpolation.
    Args:
        LF_data (numpy.ndarray): The low-frequency data array.
        LF_ts (numpy.ndarray): The corresponding low-frequency timestamps.
        HF_data (numpy.ndarray): The high-frequency data array.
        HF_ts (numpy.ndarray): The high-frequency timestamps to align to.

    Returns:
        numpy.ndarray: The LF_data interpolated onto the HF_ts timestamps.
    """
    # Check if LF timestamps are within the range of HF timestamps
    if LF_ts[0] < HF_ts[0] or LF_ts[-1] > HF_ts[-1]:
        print("Warning: LF timestamps fall outside the range of HF timestamps.")
    # aligned_LF_data = numpy.zeros_like(HF_ts)
    aligned_LF_data = numpy.interp(HF_ts, LF_ts, LF_data)
    return aligned_LF_data


# def plot_historis(key1,key2):
#     aligned_LF_fy = align_timestamp(
#         data_UR[:, header_UR[key1]],
#         data_UR[:, header_UR['Timestamp']],
#         data_FT[:, header_FT['Timestamp']]
#     )
#     # Now you can plot aligned_LF_fy against data_UR[:, header_to_index_UR['Timestamp']]
#     plt.plot(aligned_LF_fy*1e3, data_FT[:, header_FT[key2]] )
#
#     # plt.pause(10)

    def lowpass_filter(data, cutoff, order=5):
        from scipy.signal import butter, filtfilt

        if cutoff==0:
            data_p = data
        else:
            ts = data[:, 0]  # Extract the 'ts' column
            fs = 1 / numpy.mean(numpy.diff(ts))  # Calculate average time difference
            nyq = 0.5 * fs
            normal_cutoff = cutoff / nyq
            b, a = butter(order, normal_cutoff, btype='low', analog=False)
            filtered_data = numpy.apply_along_axis(lambda x: filtfilt(b, a, x), axis=0, arr=data[:, 1:8])
            data_p = numpy.concatenate((data[:, 0:1], filtered_data, data[:, 8:]), axis=1)
        return data_p

    # Example usage:
    # Assuming your data is in a NumPy array called 'data'
    # and your sampling frequency is 100 Hz, and you want to filter with a cutoff of 10 Hz



if __name__ == '__main__':

    raw_header = ["ts", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "x", "y", "z"]
    header = {raw_header: i for i, raw_header in enumerate(raw_header)}

    drag_vib_header, drag_vib = select_and_load_data("Select  File")
    drag_plain_header, drag_plain = select_and_load_data("Select  File")

    cutoff_f = 1e3
    F_drag_vib = lowpass_filter(drag_vib,       cutoff=cutoff_f)
    F_drag_plain = lowpass_filter(drag_plain,   cutoff=cutoff_f)

    key1 = 'y'
    key2 = 'Fy'
    plt.plot(F_drag_vib[:,   header[key1] ]*-1e3,F_drag_vib[:,header[key2]])
    plt.plot(F_drag_plain[:, header[key1] ]*-1e3, F_drag_plain[:, header[key2]])
    plt.xlabel("Distance (mm)")
    plt.ylabel("Force (N)")
    plt.title("Drag_force_reduction")

    plt.grid("on")
    plt.show(block=True)



    intru_vib_header, intru_vib = select_and_load_data("Select  File")
    intru_plain_header, intru_plain = select_and_load_data("Select  File")
    intru_fluid_header, intru_fluid = select_and_load_data("Select  File")

    cutoff_f = 500
    F_intru_vib = lowpass_filter(intru_vib,       cutoff=cutoff_f)
    F_intru_plain = lowpass_filter(intru_plain,   cutoff=1e3)
    F_intru_fluid = lowpass_filter(intru_fluid, cutoff=1e3)

    key1 = 'z'
    key2 = 'Fz'
    plt.plot(F_intru_vib[:,   header[key1] ]*-1e3,F_intru_vib[:,header[key2]])
    plt.plot(F_intru_plain[:, header[key1] ]*-1e3, F_intru_plain[:, header[key2]])
    plt.plot(F_intru_fluid[:, header[key1]] * -1e3,F_intru_fluid[:, header[key2]])
    plt.xlabel("Depth (mm)")
    plt.ylabel("Force (N)")
    plt.title("Intrusion_force_reduction")

    plt.grid("on")
    plt.show(block=True)