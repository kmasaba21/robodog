#!/usr/bin/python
import pickle
from os import path
import pandas as pd
import matplotlib.pyplot as plt
data_file_name="/home/masaba/.ros/motion_data.pickle"

def load_data():
    data_dict = {}
    if path.exists(data_file_name) and path.getsize(data_file_name) > 0:
        with open(data_file_name, 'rb') as fp:
            try:
                data_dict = pickle.load(fp)
            except Exception as e:
                print("error: {}".format(e))
    return data_dict

if __name__=='__main__':
    data_dict=load_data()
    data_df=pd.DataFrame(data=data_dict)
    data_df.drop_duplicates(subset=['time'],keep='first',inplace=True)
    data_df['stride'] = (data_df.right-data_df.left).abs()
    data_df.sort_values(by='time',inplace=True)
    plt.figure()
    plt.plot(data_df.time,data_df.right,'b-o')
    plt.plot(data_df.time, data_df.left,'g-o')
    plt.plot(data_df.time,data_df.stride,'r-o')
    plt.xlabel("Time (s)")
    plt.ylabel('Range(m)')
    plt.legend(['Right','Left','Stride'])
    plt.title("Left foot and Right foot Ranges and stride over time")
    plt.show()