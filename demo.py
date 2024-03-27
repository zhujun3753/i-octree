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
# # import pdb;pdb.set_trace()

OctreeMap.clear()

all_pts=0
with open('results.txt', 'w') as f:
    for i in range(20):
        print(i, file=f)
        pts = torch.randint(0,1000,size=(805285,3)).float() #* 805285
        qury_pts = torch.randint(0,1000,size=(10000,3)).float()
        t1 = time.time()
        OctreeMap.add_pts_with_attr_cpu2(pts, param)
        data2 = OctreeMap.knn_nearest_search(qury_pts, 11)
        print('ioctree',time.time()-t1, file=f)
        t2 = time.time()
        if i==0:
            all_pts = pts
            print("all_pts.shape: ", all_pts.shape, file=f)
            print("OctreeMap.get_size(): ", OctreeMap.get_size(), file=f)
        else:
            all_pts = torch.cat([all_pts, pts],dim=0)
        tree_gloabl = KDTree(all_pts, compact_nodes=False)
        min_dis, min_nnIDx = tree_gloabl.query(qury_pts, 11, workers=1)
        print('scipy',time.time()-t2, file=f)
        print('', file=f)

    print("all_pts.shape: ", all_pts.shape, file=f)
    print("OctreeMap.get_size(): ", OctreeMap.get_size(), file=f)



