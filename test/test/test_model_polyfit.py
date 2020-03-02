import numpy as np
import test_compute_path_pinv as p

#selfdrive/controls/lib/lane_planner.py--model_polyfit()

def model_polyfit(points, path_pinv):
  return np.dot(path_pinv, [float(x) for x in points])

if __name__ == '__main__':
    points = [-0.0001957019, -3.450521e-05, -0.0006073183, -0.00054086425, -0.00086613634, -0.00087764795, -0.0019247084, -0.0017599487, -0.002080593, -0.0024175213, -0.002956423, -0.0034865367, -0.0040425276, -0.0041195555, -0.0047795735, -0.0050894446, -0.0053191474, -0.0058684302, -0.0063504456, -0.0062350063, -0.0063345688, -0.0066577434, -0.0076768505, -0.009072518, -0.00921346, -0.0097392909, -0.010966797, -0.012119547, -0.012941108, -0.013490796, -0.013896115, -0.01455108, -0.014949935, -0.016045352, -0.017069425, -0.017926946, -0.018709496, -0.020198151, -0.021183133, -0.022875967, -0.023689602, -0.024834651, -0.024924252, -0.026009329, -0.027828034, -0.02797343, -0.029647391, -0.032089163, -0.032914672, -0.035047546]
    path_pinv = p.compute_path_pinv()
    # pts = [float(x) for x in points]
    # print(pts)
    # print(list(path_pinv))
    print(model_polyfit(points, path_pinv))