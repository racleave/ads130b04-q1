import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



def plot(df):
  plt.figure()

  plt.plot(df.index, df)

  plt.show()
  #import ipdb;ipdb.set_trace()


if __name__ == "__main__":

  import sys

  filename = sys.argv[1]

  df = pd.read_csv(filename, sep="\t", skiprows=10, names=['ch0', 'ch1', 'ch2', 'ch3', 'tmp']) 
  
  df.drop(columns=['tmp'], inplace=True)

  df = df.iloc[:-10,:]

  df.index = df.index.astype('int')
  df.index = df.index - df.index[0]
  
  plot(df)
