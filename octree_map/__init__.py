import torch
import os
parent_dir = os.path.dirname(os.path.abspath(__file__))
torch.classes.load_library(parent_dir+"/build/liboctree_map.so")
OctreeMap = torch.classes.octree_map.OctreeMap()
# print("Load lib end")

