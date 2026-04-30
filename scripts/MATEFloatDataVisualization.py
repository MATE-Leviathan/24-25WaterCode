# -*- coding: utf-8 -*-
"""
@description Parses time, depth, and pressure data into graphs
@author: Everett Tucker
"""

import matplotlib.pyplot as plt
import pandas as pd

test_string = "37, 2.04, 1102.573; 38, 2.3, 1022.843; 34, 2.84, 1112.934; "
def main():
    # string_hash = input("String to hash: ")
    string_hash = test_string

    times = []
    depths = []
    pressures = []

    for triplet in string_hash.strip().split(';'):
        if triplet.strip() != "":
            values = [float(x.strip()) for x in triplet.strip().split(',') if x.strip() != ""]
            times.append(values[0])
            depths.append(values[1])
            pressures.append(values[2])
                
    df = pd.DataFrame()
    df.insert(0, "time_s", times)
    df.insert(1, "depth_m", depths)
    df.insert(2, "pressure_mbar", pressures)
    
    print("\nData recieved from MATE FLoat:\n")
    print(df)
    
    plt.scatter(times, depths)
    plt.xlabel("Time (Seconds)")
    plt.ylabel("Depth (Meters)")
    plt.title("Depth vs. Time for MATE Float")
    plt.show()
    
if __name__ == '__main__':
    main()
