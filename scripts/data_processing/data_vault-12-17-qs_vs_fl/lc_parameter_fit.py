# HKJF
import tkinter as tk
from tkinter import filedialog
import numpy
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


def select_and_load_data(preload_file=None,lowpass=False,cut_off = 10, preview=False):
    import matplotlib
    matplotlib.use('TkAgg')
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
            print(f"No file selected for ''")
            return None, None
    else:
        filepath = preload_file

    # with open(filepath, 'r', newline='') as file:
        # reader = csv.reader(file)
        # headers = ["ts","Fx","Fy","Fz","Tx","Ty","Tz","x","y","z"]
        # headers = {headers: i for i, headers in enumerate(headers)}
    data = numpy.genfromtxt(filepath,delimiter=',')
    data[:,0] =  data[:,0] -  data[0,0]

    if lowpass:
        ori_force = data[:,1:4]
        cutoff_f = cut_off
        force_lf = lowpass_filter(ori_force, cutoff=cutoff_f,order=3)
        data[:, 1:4] = force_lf

    if preview:
        fig,  ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
        fig.suptitle(filepath)
        ts = data[:, 0]
        ax1.plot(data[:, 0])
        ax2.plot(ts,data[:, 1:4])
        ax3.plot(ts,data[:, 4:7])
        ax4.plot(ts,data[:, 7:10])

        plt.show()
        # plt.plot(data[:,-1])

        # for ii in range(11):
        #     if ii == 0:
        #         plt.plot(data[:, ii])
        #     else:
        #         plt.plot(data[:, 0], data[:, ii])
        #     plt.show(block=True)
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


def lowpass_filter_ORIG(data, cutoff, order=3,analog=False,plot=False):

    from scipy.signal import butter, filtfilt
    if cutoff==0:
        data_p = data
    else:
        ts = data[:, 0] # Extract the 'ts' column
        fs = 1 / numpy.mean(numpy.diff(ts)) # Calculate average time difference
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype=type, analog=analog)
        filtered_data = numpy.apply_along_axis(lambda x: filtfilt(b, a, x), axis=0, arr=data[:, 1:8])
        data_p = numpy.concatenate((data[:, 0:1], filtered_data, data[:, 8:]), axis=1)

    if plot:
        plt.plot(ts,data[:,1::])
        plt.show()
    return data_p

def lowpass_filter(data, cutoff, order=3, analog=False, plot=False):
    import numpy
    from scipy.signal import butter, sosfilt, sosfilt_zi, zpk2sos, welch
    import matplotlib.pyplot as plt

    if cutoff == 0:
        data_p = data
    else:
        ts = data[:, 0]  # Extract the 'ts' column
        fs = 1 / numpy.mean(numpy.diff(ts))  # Calculate average sampling frequency
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        z, p, k = butter(order, normal_cutoff, btype='low', analog=analog, output='zpk')
        sos = zpk2sos(z, p, k)

        # Apply filtering to the single channel
        zi = sosfilt_zi(sos) * data[0, 1]
        filtered_data, _ = sosfilt(sos, data[:, 1], zi=zi)

        data_p = numpy.concatenate((data[:, 0:1], filtered_data.reshape(-1, 1), data[:, 2:]), axis=1)

    if plot:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # Time-domain plot
        ax1.plot(ts, data[:, 1], label='Original')
        ax1.plot(ts, data_p[:, 1], label='Filtered')
        ax1.set_title('Original and Filtered Signal (Single Channel)')
        ax1.set_ylabel('Amplitude')
        ax1.legend()

        # Power spectrum plot
        f, Pxx_orig = welch(data[:, 1], fs, nperseg=1024)
        f, Pxx_filt = welch(data_p[:, 1], fs, nperseg=1024)
        ax2.semilogy(f, Pxx_orig, label='Original', alpha=0.7)
        ax2.semilogy(f, Pxx_filt, label='Filtered', alpha=0.7)
        ax2.axvline(cutoff, color='red', linestyle='--', label='Cutoff')
        ax2.set_ylabel('Power/Frequency')
        ax2.set_xlabel('Frequency (Hz)')
        ax2.legend()

        plt.suptitle('Low-pass Filtered Data and Power Spectrum (Single Channel)')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout for title
        plt.show()

    return data_p

if __name__ == '__main__':

    raw_header = ["ts", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "x", "y", "z","isMoving"]
    header = {raw_header: i for i, raw_header in enumerate(raw_header)}

    aoa_00_qs = select_and_load_data("2024-12-17-09-59_fin_qs_aoa_0.csv",preview=True)
    aoa_30_qs = select_and_load_data("2024-12-17-10-07_fin_qs_aoa_30.csv",preview=True)
    aoa_60_qs = select_and_load_data("2024-12-17-10-13_fin_qs_aoa_60.csv",preview=True)
    aoa_90_qs = select_and_load_data("2024-12-17-10-24_fin_qs_aoa_90.csv",preview=True)

    aoa = [0,30,60,90]
    Fx = [0.4,11.6,15.8,18]
    Fy = [12,7.4,4.4,-1.8]
    body_force_y = [12.67,8.4,8.01]
    body_force_x =[4.08,2.15,3.87]


    aoa_90_lf = select_and_load_data("2024-12-17-11-15_fin_lf_aoa_90.csv",lowpass=True,cut_off=2e3,preview=True)
    aoa_60_lf = select_and_load_data("2024-12-17-11-08_fin_lf_aoa_60.csv",lowpass=True,cut_off=1e3,preview=True)
    aoa_30_lf = select_and_load_data("2024-12-17-11-03_fin_lf_aoa_30.csv",lowpass=True,cut_off=1e3,preview=True)
    aoa_00_lf = select_and_load_data("2024-12-17-10-58_fin_lf_aoa_0.csv",lowpass=True,cut_off=1e3,preview=True)

    aoa_90_Fx = aoa_90_lf[:,0:2]
    aoa_60_Fx = aoa_60_lf[:,0:2]
    aoa_30_Fx = aoa_30_lf[:,0:2]
    aoa_00_Fx = aoa_00_lf[:,0:2]

    aoa_90_Fx_filtered = lowpass_filter(aoa_90_Fx,10,order=3,analog=False,plot=True)
    aoa_60_Fx_filtered = lowpass_filter(aoa_60_Fx,50,order=3,analog=False,plot=True)
    aoa_30_Fx_filtered = lowpass_filter(aoa_30_Fx,50,order=3,analog=False,plot=True)
    aoa_00_Fx_filtered = lowpass_filter(aoa_00_Fx,10,order=3,analog=False,plot=True)
    
    body_qs = select_and_load_data("2024-12-17-12-18_body_qs_aoa_0.csv",lowpass=True,cut_off=2e3,preview=True)
    body_lf_1 = select_and_load_data("2024-12-17-11-50_body_lf_1_aoa_0.csv",lowpass=False,cut_off=1e3,preview=True)
    body_lf_2 = select_and_load_data("2024-12-17-12-13_body_lf_2_aoa_0.csv",lowpass=False,cut_off=1e3,preview=True)

    body_lf_1_f = body_lf_1[:,[0,2]]
    body_lf_2_f = body_lf_2[:,[0,2]]
    body_lf_1_f = lowpass_filter(body_lf_1_f, 10, order=3, analog=False, plot=True)
    body_lf_2_f = lowpass_filter(body_lf_2_f, 10, order=3, analog=False, plot=True)

    body_force_y = [12.67,8.4,8.01]
    body_force_x =[4.08,2.15,3.87]
    
    key1 = 'y'
    key2 = 'Fx'
    # plt.plot(aoa_90_qs_force[:,   header[key1] ]*-1e3,aoa_90_qs[:,header[key2]])
    # plt.plot(aoa_90_lf_force[:, header[key1] ]*-1e3, aoa_90_lf[:, header[key2]])


    # plt.grid("on")
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