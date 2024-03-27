import torch
from octree_map import OctreeMap
from scipy.spatial import KDTree
import time

OctreeMap.debug_print()
pts = torch.rand((100,9)).float()
param = torch.tensor([1,2,3,4]).float()
OctreeMap.add_pts_with_attr_cpu(pts, param)
data = OctreeMap.get_data()
query_pts = torch.rand((2,9)).float()
data2 = OctreeMap.nearest_search(query_pts, param)
data3 = OctreeMap.knn_nearest_search(query_pts, 5)
data4 = OctreeMap.radius_neighbors(query_pts, 0.1)

print(pts[0])
print(data2[0])
# import pdb;pdb.set_trace()



