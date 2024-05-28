
import os
import pandas as pd

# Set the directory containing your CSV files
folder_path = '/home/roboticme/catkin_thesis_ws/src/disassembly-station/screw_alignment/screw_alignment_data'

# Get a list of all CSV files in the folder
csv_files = [file for file in os.listdir(folder_path) if file.endswith('.csv')]

# Initialize an empty DataFrame to store the merged data
merged_data = pd.DataFrame()

# Read the first file to get the common header
first_file_path = os.path.join(folder_path, csv_files[0])
common_header = pd.read_csv(first_file_path, header=None, nrows=1)

# Iterate through each CSV file and merge its data into the DataFrame
for file in csv_files:
    file_path = os.path.join(folder_path, file)
    df = pd.read_csv(file_path, header=None, skiprows=1)  # Skip the first row (common header)
    merged_data = pd.concat([merged_data, df], axis=0, ignore_index=True)

# Concatenate the common header to the merged data
merged_data = pd.concat([common_header, merged_data], axis=0, ignore_index=True)

# Write the merged DataFrame to a new CSV file
output_path = '/home/roboticme/catkin_thesis_ws/src/disassembly-station/screw_alignment/screw_alignment_data/merged_data.csv'
merged_data.to_csv(output_path, index=False, header=None)  # Assuming you don't want a header in the output file
