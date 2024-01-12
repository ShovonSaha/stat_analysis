import numpy as np

# Initializing boundaries as percentages
a = 0.25 # percentage

first_col_value = int(4 * a)
# first_col = reshaped_array[:, :first_col_value]
# last_col = reshaped_array[:, -first_col_value:]

# Example rectangular array
array = np.array([[1, 2, 3, 4, 5],
                  [6, 7, 8, 9, 10],
                  [11, 12, 13, 14, 15],
                  [16, 17, 18, 19, 20]])

# Calculate the mean and standard deviation for the middle section
middle_section = array[first_col_value:-first_col_value, first_col_value:-first_col_value]  # Exclude first and last rows and columns

middle_avg = np.mean(middle_section)
middle_std = np.std(middle_section)

print(middle_section)
print("mean_middle_section:", middle_avg)
print("std_middle_section:", middle_std)