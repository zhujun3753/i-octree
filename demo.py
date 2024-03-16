import torch
from octree_map import OctreeMap

OctreeMap.debug_print()
pts = torch.rand((100,9)).float()
param = torch.tensor([1,2,3,4]).float()
OctreeMap.add_pts_with_attr_cpu(pts, param)
data = OctreeMap.get_data()
data2 = OctreeMap.nearest_search(pts, param)
print(pts[0])
print(data2[0])
# import pdb;pdb.set_trace()