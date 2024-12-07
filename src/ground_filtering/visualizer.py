from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas

def main():
  points = pandas.read_csv('point_clouds/parsed_point_cloud_50.csv')

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  x = points['y'].values
  y = points['z'].values
  z = points['z'].values

  #  x   y   z  
  # 1.1,1.2,1.3
  # 2.1,2.2,2.3
  # 3.1,3.2,3.3
  # 4.1,4.2,4.3

  ax.scatter(x, y, z, c='r', marker='o')

  plt.show()
  
  points2 = pandas.read_csv('point_clouds/vis_point_cloud_50.csv')

  fig2 = plt.figure()
  ax2 = fig2.add_subplot(111, projection='3d')

  x2 = points2['y'].values
  y2 = points2['z'].values
  z2 = points2['x'].values

  #  x   y   z  
  # 1.1,1.2,1.3
  # 2.1,2.2,2.3
  # 3.1,3.2,3.3
  # 4.1,4.2,4.3

  ax2.scatter(x2, y2, z2, c='r', marker='o')

  plt.show()

if __name__ == "__main__":
  main()