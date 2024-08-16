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
        headers = next(reader)
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


def plot_historis(key1,key2):
    aligned_LF_fy = align_timestamp(
        data_UR[:, header_UR[key1]],
        data_UR[:, header_UR['Timestamp']],
        data_FT[:, header_FT['Timestamp']]
    )
    # Now you can plot aligned_LF_fy against data_UR[:, header_to_index_UR['Timestamp']]
    plt.plot(aligned_LF_fy*1e3, data_FT[:, header_FT[key2]] )

    # plt.pause(10)


if __name__ == '__main__':

    header_FT, data_FT = select_and_load_data("SelectS FT File")
    header_UR, data_UR = select_and_load_data("Select UR File")

        # # plt.plot(COMBO_ts)
        # plt.plot(data_FT[:,0])
        # plt.plot(data_UR[:,0])

        ## Check if both files were selected successfully
    if data_FT is not None and data_UR is not None:
        # Plotting: Data Exploration
        plt.ion()
        key1 = "X"
        plot_historis(key1, "Fx")
        plot_historis(key1, "Fy")
        plot_historis(key1, "Fz")

        plt.xlabel("Distance (mm)")
        plt.ylabel("Force (N)")
        plt.legend(["Fx","Fy","Fz"])
        plt.show(block=True)

        # plt.pause(50)
    else:
        header_COMOBO,data_COMOBO = header_FT, data_FT
        COMBO_ts =  data_COMOBO[:,0]
        key1 = "X"
        plt.plot(1e3*data_COMOBO[:,header_COMOBO[key1]],data_COMOBO[:,header_COMOBO["Fx"]])
        plt.plot(1e3*data_COMOBO[:,header_COMOBO[key1]],data_COMOBO[:,header_COMOBO["Fy"]])
        plt.plot(1e3*data_COMOBO[:,header_COMOBO[key1]],data_COMOBO[:,header_COMOBO["Fz"]])
        plt.show(block=True)


