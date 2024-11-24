# import csv
# import matplotlib.pyplot as plt
#
# # Read the data from the CSV file
# x = []
# y = []
#
# # Replace 'your_data.csv' with your actual file
# with open('OCT27-DL_2point_heat30_3.csv', newline='') as csvfile:
#     reader = csv.reader(csvfile)
#     next(reader)  # Skip the header row if your CSV has one
#     for row in reader:
#         x.append(float(row[3]))  # 3rd column (index 2)
#         y.append(float(row[7]))  # 7th column (index 6)
#
# # Convert y to negative and scale
# y_neg_scaled = [-value * 1e3 for value in y]
#
# # Create the plot
# plt.figure(figsize=(10, 6))
# plt.plot(y_neg_scaled, x, marker='o', linestyle='-', color='b')
# plt.title('Plot of Column 3 vs Column 7')
# plt.xlabel('Column 3')
# plt.ylabel('Column 7')
# plt.grid(True)
# plt.show()

import csv
import matplotlib.pyplot as plt

def read_data(filename):
    x = []
    y = []
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header row if your CSV has one
        for row in reader:
            try:
                x.append(float(row[3]))  # 3rd column
                y.append(float(row[7]))  # 7th column
            except ValueError:
                print(f"Skipping row due to ValueError: {row}")
    return x, y

# Read data from three files
# file1 = 'OCT27-DL_2point_heat40_1.csv'
# file2 = 'OCT27-DL_2point_heat40_2.csv'
# file3 = 'OCT29-DL_2point_heat40_1.csv'
file1 = 'OCT27-DL_2point_heat30_1.csv'
file2 = 'OCT27-DL_2point_heat30_2.csv'
file3 = 'OCT27-DL_2point_heat30_3.csv'

x1, y1 = read_data(file1)
x2, y2 = read_data(file2)
x3, y3 = read_data(file3)

# Convert y to negative and scale
y1_neg_scaled = [-value * 1e3 for value in y1]
y2_neg_scaled = [-value * 1e3 for value in y2]
y3_neg_scaled = [-value * 1e3 for value in y3]

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(y1_neg_scaled, x1, marker='o', linestyle='-', color='b', label='File 1')
plt.plot(y2_neg_scaled, x2, marker='o', linestyle='-', color='g', label='File 2')
plt.plot(y3_neg_scaled, x3, marker='o', linestyle='-', color='r', label='File 3')
# plt.show()

file1 = 'OCT27-DL_2point_heat40_1.csv'
file2 = 'OCT27-DL_2point_heat40_2.csv'
file3 = 'OCT29-DL_2point_heat40_1.csv'

x1, y1 = read_data(file1)
x2, y2 = read_data(file2)
x3, y3 = read_data(file3)

# Convert y to negative and scale
y1_neg_scaled = [-value * 1e3 for value in y1]
y2_neg_scaled = [-value * 1e3 for value in y2]
y3_neg_scaled = [-value * 1e3 for value in y3]

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(y1_neg_scaled, x1, marker='o', linestyle='-', color='b', label='File 1')
plt.plot(y2_neg_scaled, x2, marker='o', linestyle='-', color='g', label='File 2')
plt.plot(y3_neg_scaled, x3, marker='o', linestyle='-', color='r', label='File 3')
plt.show()