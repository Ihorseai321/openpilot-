from test_interp import interp
from test_compute_path_pinv import compute_path_pinv
from test_model_polyfit import model_polyfit

#selfdrive/controls/lib/lane_planner.py--calc_d_poly()

def calc_d_poly(l_poly, r_poly, p_poly, l_prob, r_prob, lane_width):
  # This will improve behaviour when lanes suddenly widen
  lane_width = min(4.0, lane_width)
  l_prob = l_prob * interp(abs(l_poly[3]), [2, 2.5], [1.0, 0.0])
  r_prob = r_prob * interp(abs(r_poly[3]), [2, 2.5], [1.0, 0.0])

  path_from_left_lane = l_poly.copy()
  path_from_left_lane[3] -= lane_width / 2.0
  path_from_right_lane = r_poly.copy()
  path_from_right_lane[3] += lane_width / 2.0

  lr_prob = l_prob + r_prob - l_prob * r_prob
  print(path_from_left_lane, path_from_right_lane)

  d_poly_lane = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
  return lr_prob * d_poly_lane + (1.0 - lr_prob) * p_poly

if __name__ == '__main__':
    CAMERA_OFFSET = 0.06
    v_ego = 20
    path_pinv = compute_path_pinv()
    leftLane_points = [1.5761895, 1.574351, 1.5732352, 1.5744189, 1.573061, 1.5705775, 1.5694419, 1.569258, 1.5683932, 1.5667254, 1.5641075, 1.5629115, 1.5624135, 1.5615864, 1.5606759, 1.5605713, 1.5588325, 1.5576929, 1.555786, 1.5534463, 1.5528187, 1.5517489, 1.5509077, 1.5491813, 1.5485182, 1.546626, 1.5463024, 1.5436103, 1.54093, 1.5397468, 1.5380195, 1.5362276, 1.5343323, 1.5322795, 1.5312299, 1.529554, 1.5268264, 1.5261678, 1.5240464, 1.5228835, 1.5207725, 1.5199273, 1.5197134, 1.5178593, 1.5164641, 1.514294, 1.5133548, 1.5123382, 1.5103731, 1.5075383]
    rightLane_points = [-1.893837, -1.894454, -1.8966849, -1.8974499, -1.8997052, -1.900787, -1.9027631, -1.9037013, -1.9065424, -1.9074453, -1.9093661, -1.9096339, -1.9100418, -1.9111718, -1.9121888, -1.9136063, -1.9147706, -1.9156369, -1.9183264, -1.9199908, -1.9218354, -1.9238859, -1.9261016, -1.92621, -1.9274943, -1.9293803, -1.9307779, -1.9334117, -1.9356186, -1.9379534, -1.9396253, -1.9411077, -1.9430276, -1.9443495, -1.9440197, -1.9461031, -1.9482435, -1.9502844, -1.9523971, -1.954372, -1.9568144, -1.9574634, -1.9596272, -1.9618131, -1.9654566, -1.9675995, -1.9706581, -1.9724663, -1.9744929, -1.97766]
    path_points = [-0.0001957019, -3.450521e-05, -0.0006073183, -0.00054086425, -0.00086613634, -0.00087764795, -0.0019247084, -0.0017599487, -0.002080593, -0.0024175213, -0.002956423, -0.0034865367, -0.0040425276, -0.0041195555, -0.0047795735, -0.0050894446, -0.0053191474, -0.0058684302, -0.0063504456, -0.0062350063, -0.0063345688, -0.0066577434, -0.0076768505, -0.009072518, -0.00921346, -0.0097392909, -0.010966797, -0.012119547, -0.012941108, -0.013490796, -0.013896115, -0.01455108, -0.014949935, -0.016045352, -0.017069425, -0.017926946, -0.018709496, -0.020198151, -0.021183133, -0.022875967, -0.023689602, -0.024834651, -0.024924252, -0.026009329, -0.027828034, -0.02797343, -0.029647391, -0.032089163, -0.032914672, -0.035047546]
    l_poly = model_polyfit(leftLane_points, path_pinv)  # left line
    r_poly = model_polyfit(rightLane_points, path_pinv)  # right line
    p_poly = model_polyfit(path_points, path_pinv)  # predicted path
    l_prob = 0.9766466  # left line prob
    r_prob = 0.97837639  # right line prob
    
    l_poly[3] += CAMERA_OFFSET
    r_poly[3] += CAMERA_OFFSET
    
    lane_width_estimate = 3.7
    lane_width_certainty = 1.0
    lane_width = 3.7
    # Find current lanewidth
    lane_width_certainty += 0.05 * (l_prob * r_prob - lane_width_certainty)
    # print(path_pinv)
    current_lane_width = abs(l_poly[3] - r_poly[3])
    lane_width_estimate += 0.005 * (current_lane_width - lane_width_estimate)
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    lane_width = lane_width_certainty * lane_width_estimate + \
                      (1 - lane_width_certainty) * speed_lane_width

    d_poly = calc_d_poly(l_poly, r_poly, p_poly, l_prob, r_prob, lane_width)
    print(d_poly)