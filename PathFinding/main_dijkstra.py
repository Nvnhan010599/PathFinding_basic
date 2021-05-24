from MapAndObs import map_fullIndex
from MapAndObs import lbx, ubx, lby, uby
from MapAndObs import step_idx, step_idy # 71, 61
from MapAndObs import resolution, robot_radius
from MapAndObs import bwx,bwy
import math
import matplotlib.pyplot as plt
from library_dijkstra import Dijkstra_Algorithm

def main_dijkstra(Start,Target):
    dijkstra = Dijkstra_Algorithm(Start,Target,resolution,robot_radius)
    dijkstra.get_map(map_fullIndex)
    dijkstra.get_map_para(lbx,ubx,lby,uby,step_idx,step_idy)
    dijkstra.get_wall(bwx,bwy)
    dijkstra.plot_Map()
    while(dijkstra.k != dijkstra.target_vertex):
        k = dijkstra.k
        ix0 = int(k/step_idy)
        iy0 = int(k%step_idy)
        xk = lbx + ix0*resolution
        yk = lby + iy0*resolution
        dijkstra.create_listIndex_check(ix0,iy0,xk,yk)
        dijkstra.calculus_Dis(xk,yk)
        idmin = dijkstra.min_of_U()
        dijkstra.re_init(idmin)

        plt.plot(xk,yk,"r.")

        plt.pause(0.001)
 
    path = dijkstra.result()
    dijkstra.plot_Path()
    
    return path
start_point = [1,1,0,0,0] # input
target_point = [5.6,1.1] # input
# for i in range(step_idx):
#     for j in range(step_idy):
#         t_x = lbx + i*resolution
#         t_y = lby + j*resolution
#         if(map_fullIndex[i][j] == 0):
#             plt.plot(t_x,t_y,'r,')
#             pass
#         # if(map_fullIndex[i][j] == -1):
#         #     # plt.plot(t_x,t_y,'r,')
#         #     pass

Target = main_dijkstra(start_point[:2],target_point)
for i in range(step_idx):
    for j in range(step_idy):
        t_x = lbx + i*resolution
        t_y = lby + j*resolution
        if(map_fullIndex[i][j] == 0):
            plt.plot(t_x,t_y,'r,')
            pass

plt.legend()
plt.show()