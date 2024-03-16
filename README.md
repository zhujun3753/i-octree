# [i-Octree: A Fast, Lightweight,  and Dynamic  Octree for Proximity  Search(Accepted by ICRA 2024)](https://arxiv.org/abs/2309.08315)

**i-Octree** is a dynamic octree data structure that supports both fast nearest neighbor search and real-time dynamic updates, such as point insertion, deletion, and on-tree down-sampling. The i-Octree is built upon a leaf-based octree and has two key features: a local spatially continuous storing strategy that allows for fast access to points while minimizing memory usage, and local on-tree updates that significantly reduce computation time compared to existing static or dynamic tree structures. 

## Features
- Dynamically insert points to the tree.
- Delete points inside given axis-aligned bounding boxes.
- Fast k-nearest neighbors search.
- Fast radius neighbors search.
- Fully templated for maximal flexibility to support arbitrary point representations & containers.

## News 📰
### [2024.03.16] - Feature Enhancement
- Enhanced the implementation of the i-Octree with new functionalities and updated the Python bindings accordingly.

## Python Bindings Test
### 1. Requirement
To compile, we require `Eigen`, `C++17`, and `torch`.
### 2. Run
```bash
git clone git@github.com:zhujun3753/i-octee.git

cd octree_map
# Update the `CMAKE_PREFIX_PATH` variable in `CMakeLists.txt` to reflect your own path settings.!!!!
bash build.sh

cd ..
python demo.py

```
### 3. Results
```shell
==============================
This is a debug print in OctreeMap C++!
==============================
num: 100
attr_n: 6
after filter num: 95
octree_feature.get_size(): 85
tensor([0.5879, 0.8644, 0.9247, 0.9912, 0.9457, 0.2752, 0.5103, 0.7180, 0.9304])
tensor([0.5879, 0.8644, 0.9247, 0.9912, 0.9457, 0.2752, 0.5103, 0.7180, 0.9304])
```

## Run Randomized Data Experiments

### 1. Build
```shell
git clone git@github.com:zhujun3753/i-octee.git

# For Comparison
cd i-octree
git clone git@github.com:hku-mars/ikd-Tree.git

# Build & Run
bash run.sh

# Plot Results
python plot_time.py

```


###  2. Results

![](examples/output/figures/random.png)


## Attribution

If you use the implementation or ideas from the [corresponding paper](https://arxiv.org/abs/2309.08315) in your academic work, it would be nice if you cite the corresponding paper:
```
@misc{zhu2023ioctree,
      title={i-Octree: A Fast, Lightweight, and Dynamic Octree for Proximity Search}, 
      author={Jun Zhu and Hongyi Li and Shengjie Wang and Zhepeng Wang and Tao Zhang},
      year={2023},
      eprint={2309.08315},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Acknowledgement

Thanks to Jens Behley for open-sourcing his excellent work [octree](https://github.com/jbehley/octree). 

This project uses "ikd-Tree" by Cai, Yixi for comparison purposes. The code from "ikd-Tree" is licensed under the GPL-2.0. You can find the original project and its source code [here](https://github.com/hku-mars/ikd-Tree).

## License

The source code of i-Octree is released under [GPLv2](http://www.gnu.org/licenses/old-licenses/gpl-2.0.html) license. For commercial use, please contact Mr. Jun ZHU (<j-zhu20@mails.tsinghua.edu.cn>) or Dr. Tao ZHANG (<taozhang@tsinghua.edu.cn>).

