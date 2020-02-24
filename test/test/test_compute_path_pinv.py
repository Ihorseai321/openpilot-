#selfdrive/controls/lib/lane_planner.py--compute_path_pinv()
import numpy as np

def compute_path_pinv(l=50):
  deg = 3
  x = np.arange(l*1.0)
  X = np.vstack(tuple(x**n for n in range(deg, -1, -1))).T
  # print(X)
  pinv = np.linalg.pinv(X)
  return pinv

if __name__ == "__main__":
  print(compute_path_pinv())
