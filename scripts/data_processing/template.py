# HKJF
import tkinter as tk
from tkinter import filedialog
import numpy
import matplotlib.pyplot as plt
import csv

def select_and_load_data(preload_file=None,preview=False):
    def plot_data(data, key1='ts', key2='Fx'):
        raw_header = ["ts", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "x", "y", "z", "isMoving"]
        header = {raw_header: i for i, raw_header in enumerate(raw_header)}
        plt.plot(data[:, header[key1]], data[:, header[key2]])

    root = tk.Tk()
    root.withdraw()

    if preload_file is None:
        filepath = filedialog.askopenfilename(
            initialdir=".",
            title="Select CSV File",
            filetypes=(("CSV Files", "*.csv"), ("All Files", "*.*"))
        )

        if not filepath:
            print(f"No file selected for '{file_title}'")
            return None, None
    else:
        filepath = preload_file

    # with open(filepath, 'r', newline='') as file:
        # reader = csv.reader(file)
        # headers = ["ts","Fx","Fy","Fz","Tx","Ty","Tz","x","y","z"]
        # headers = {headers: i for i, headers in enumerate(headers)}
    data = numpy.genfromtxt(filepath,delimiter=',')
    data[:,0] =  data[:,0] -  data[0,0]

    if preview:
        for ii in range(11):
            if ii == 0:
                plt.plot(data[:, ii])
            else:
                plt.plot(data[:, 0], data[:, ii])
            plt.show(block=True)
    return data

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

if __name__ == '__main__':

    raw_header = ["ts", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "x", "y", "z","isMoving"]
    header = {raw_header: i for i, raw_header in enumerate(raw_header)}

    aoa_0_qs = select_and_load_data("data_vault-12-17-qs_vs_fl/2024-12-17-09-59_fin_qs_aoa_0.csv")
    aoa_0_lf = select_and_load_data("2024-12-17-10-07_fin_qs_aoa_30.csv")

    cutoff_f = 1e3


    F_drag_vib = lowpass_filter(aoa_0_qs[0:,1:4],       cutoff=cutoff_f)
    F_drag_plain = lowpass_filter(aoa_0_lf[0,1:4],   cutoff=cutoff_f)

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