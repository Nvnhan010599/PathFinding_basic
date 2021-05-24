from MapAndObs import map_fullIndex
from MapAndObs import lbx, ubx, lby, uby
from MapAndObs import step_idx, step_idy # 71, 61
from MapAndObs import resolution, robot_radius
from MapAndObs import bwx,bwy
import math
import matplotlib.pyplot as plt
from library_A_star import A_star_Algorithm

#

#

def main_a_start(Start,Target):
    a_start = A_star_Algorithm(Start,Target,resolution,robot_radius)
    a_start.get_map(map_fullIndex)
    a_start.get_map_para(lbx,ubx,lby,uby,step_idx,step_idy)
    a_start.get_wall(bwx,bwy)
    a_start.plot_Map()
    while(a_start.k != a_start.target_vertex):
        k = a_start.k
        ix0 = int(k/step_idy)
        iy0 = int(k%step_idy)
        xk = lbx + ix0*resolution
        yk = lby + iy0*resolution
        a_start.create_listIndex_check(ix0,iy0,xk,yk)
        a_start.calculus_Dis(xk,yk)
        idmin = a_start.min_of_U()
        a_start.re_init(idmin)

        plt.plot(xk,yk,"r.")

        plt.pause(0.0001)
 
    path = a_start.result()
    a_start.plot_Path()
    
    return path
start_point = [1,1,0,0,0] # input
target_point = [5.6,1.1] # input


Target = main_a_start(start_point[:2],target_point)
for i in range(step_idx):
    for j in range(step_idy):
        t_x = lbx + i*resolution
        t_y = lby + j*resolution
        if(map_fullIndex[i][j] == 0):
            plt.plot(t_x,t_y,'r,')
#             pass
#         # if(map_fullIndex[i][j] == -1):
#         #     # plt.plot(t_x,t_y,'r,')
#         #     pass
plt.legend()
plt.show()
# 