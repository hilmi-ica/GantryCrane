import cv2
import numpy as np
import matplotlib.pyplot as plt

def convert1(depth_map):
    # return -71.93 * depth_map + 130.4 # dpt raw
    return -71.28311611 * depth_map + 131.18177774 # dpt
    # return 93.61562577 * depth_map + 63.35978286 # fastdepth
    # return 13.1393517 * depth_map + 73.15217372 # monodepth2
    # return -57.92661515 * depth_map + 105.10912245 # midas
    # return 48.29909342 * depth_map + 47.48710793 # hybrid

def convert2(depth_map):
    # x = [depth_map[807,80], depth_map[206,159], depth_map[890,966], depth_map[492,977], depth_map[166,980], depth_map[828,1899], depth_map[263,1851]]
    # y = [115.7881687, 132.5243563, 99.33253495, 105.029484, 118.1905453, 116.3312189, 132.7968561]
    x = [depth_map[852,374], depth_map[875,628], depth_map[893,914], depth_map[887,1011], depth_map[876,1286], depth_map[867,1544]]
    y = [109.4652913, 102.1892852, 99.40367196, 99.38254374, 102.119244, 108.7008395]
    calib = np.poly1d(np.polyfit(x,y,1))
    # myline = np.linspace(min(x), max(x), 100)
    # plt.scatter(x, y)
    # plt.plot(myline, calib(myline))
    # plt.show()
    a, b = calib.coefficients
    return a, b, a * depth_map + b

def convert3(depth_map):
    # return 0.69718971 * depth_map + 1.05980466 # fastdepth
    # return -3.66993308 * depth_map + 2.44481947 # monodepth2
    # return -2.48448415 * depth_map + 2.32151516 # midas
    # return -2.33010559 * depth_map + 1.9679718 # hybrid
    return -2.71617353 * depth_map + 2.49307072 # dpt

# Calib Gantry-2
def convert8(depth_map):
    # return 6051.6 * depth_map**6 - 13789 * depth_map**5 + 8428.7 * depth_map**4 + 2131 * depth_map**3 - 4047 * depth_map**2 + 1347.2 * depth_map - 40.039 # DPT
    return 31.966 * depth_map**2 - 95.035 * depth_map + 118.81 # DPT-1
    # return 43332 * depth_map**6 - 134479 * depth_map**5 + 163292 * depth_map**4 - 98269 * depth_map**3 + 30524 * depth_map**2 - 4599.7 * depth_map + 349.63 # MiDaS
    # return -145.34 * depth_map**3 + 167.68 * depth_map**2 - 82.441 * depth_map + 100.72 # MiDaS-1
    # return -89162134 * depth_map**6 + 77894962 * depth_map**5 - 26321347 * depth_map**4 + 4351358 * depth_map**3 - 368859 * depth_map**2 + 15274 * depth_map - 163 #FastDepth
    # return 127988 * depth_map**4 - 58699 * depth_map**3 + 5957.2 * depth_map**2 + 109.73 * depth_map + 61.292 # FastDepth-1
    # return -94.006 * depth_map**3 + 141.9 * depth_map**2 - 82.851 * depth_map + 98.754 # Monodepth2
    # return -35.672 * depth_map**2 + 22.543 * depth_map + 79.364 # Monodepth2-1

def distance(depth_metric):
   
    obj_dist = np.array([
        [depth_metric[527,251], depth_metric[524,500], depth_metric[532,810], depth_metric[532,1123], depth_metric[538,1425], depth_metric[527,1679]],
        [depth_metric[459,163], depth_metric[461,426], depth_metric[452,787], depth_metric[438,1141], depth_metric[455,1500], depth_metric[469,1767]],
        [depth_metric[384,56], depth_metric[361,337], depth_metric[347,735], depth_metric[347,1185], depth_metric[363,1593], depth_metric[392,1882]],
        [depth_metric[309,0], depth_metric[239,205], depth_metric[255,675], depth_metric[239,1246], depth_metric[240,1717], depth_metric[307,1919]]
    ])

    return obj_dist

def distance1(depth_metric,d):
    obj_dist = np.array([
        [np.mean(depth_metric[527-d:527+d,251-d:251+d]), np.mean(depth_metric[524-d:524+d,500-d:500+d]), np.mean(depth_metric[532-d:532+d,810-d:810+d]), np.mean(depth_metric[532-d:532+d,1123-d:1123+d]), np.mean(depth_metric[538-d:538+d,1425-d:1425+d]), np.mean(depth_metric[527-d:527+d,1679-d:1679+d])],
        [np.mean(depth_metric[459-d:459+d,163-d:162+d]), np.mean(depth_metric[461-d:461+d,426-d:426+d]), np.mean(depth_metric[452-d:452+d,787-d:787+d]), np.mean(depth_metric[438-d:438+d,1141-d:1141+d]), np.mean(depth_metric[455-d:455+d,1500-d:1500+d]), np.mean(depth_metric[469-d:469+d,1767-d:1767+d])],
        [np.mean(depth_metric[384-d:384+d,56-d:56+d]), np.mean(depth_metric[361-d:361+d,337-d:337+d]), np.mean(depth_metric[347-d:347+d,735-d:735+d]), np.mean(depth_metric[347-d:347+d,1185-d:1185+d]), np.mean(depth_metric[363-d:363+d,1593-d:1593+d]), np.mean(depth_metric[392-d:392+d,1882-d:1882+d])],
        [np.mean(depth_metric[309-d:309+d,0:0+d]), np.mean(depth_metric[239-d:239+d,205-d:205+d]), np.mean(depth_metric[255-d:255+d,675-d:675+d]), np.mean(depth_metric[239-d:239+d,1246-d:1246+d]), np.mean(depth_metric[240-d:240+d,1717-d:1717+d]), np.mean(depth_metric[307-d:307+d,1919-d:1919])]
    ])

    return obj_dist

def vacancies(dist, err=0):
   
    # setup
    space = np.full(dist.shape, err)
    # thresh = np.array([
    #     [103, 94.4, 89.5, 89.6, 94, 102.8],
    #     [92, 81.5, 75.8, 76.4, 81.5, 91.4],
    #     [81.1, 69.6, 63.1, 62.7, 69.4, 79.9],
    #     [75.6, 59.6, 50.2, 50.9, 59, 75]
    # ])
    thresh = np.array([
        [104.1460063, 94.94677785, 90.47475131, 90.40758057, 94.85043819, 103.2235469],
        [93.04187565, 82.61531713, 77.43436333, 77.35587001, 82.50457942, 92.00815521],
        [82.82023077, 70.90621006, 64.79568369, 64.70185952, 70.77715468, 81.65721416],
        [69.28347566, 60.183807, 52.84771163, 52.7326334, 60.03170516, 68.06924783]
    ])

    # calculate
    vacant = (thresh + space) - dist
    for i in range(vacant.shape[0]):
        for j in range(vacant.shape[1]):
            if vacant[i,j] >= 0:
                vacant[i,j] = 1
            else:
                vacant[i,j] = 0
    
    return vacant

def locate(vct_idx):

    # setup
    x_dist = [0.235, 0.440, 0.645, 0.850, 1.055, 1.260]
    # x_dist = [0.235, 0.450, 0.65, 0.855, 1.06, 1.265]
    y_dist = [0.64, 0.52, 0.39, 0.25]
    y_real = [0, 0.1406, 0.2812, 0.4218]

    # location
    for i in range(vct_idx.shape[0]):
        for j in range(vct_idx.shape[1]):
            if vct_idx[i,j] == 0:
                idx = j
                idy = i
                setpoint_x = x_dist[idx]
                setpoint_y = y_dist[idy]
                return setpoint_x, setpoint_y, y_real[idy]
