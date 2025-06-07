import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("accelerometer_test_data/filtered_data.csv", header = 2)

df.plot(x="sample", y = "Raw Accel",xlabel= "sample", ylabel= "g's")
df.plot(x = "sample", y = "Filtered Accel", ylabel = "g's")
plt.show()




