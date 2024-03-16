// Copyright (c) 2023 Jun Zhu, Tsinghua University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <map>

#include <torch/custom_class.h>
#include <torch/script.h>
#include "Octree.hpp"
#include <Eigen/Eigen>


#ifndef OCTREEMAP_H_INCLUDED
#define OCTREEMAP_H_INCLUDED

#define HASH_P 116101
#define MAX_N 10000000000
class VOXEL_LOC
{
public:
	int64_t x, y, z;

	VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
		: x(vx), y(vy), z(vz) {}

	bool operator==(const VOXEL_LOC &other) const
	{
		return (x == other.x && y == other.y && z == other.z);
	}

    bool operator<(const VOXEL_LOC &other) const
	{
		return (x < other.x ||  x == other.x && y < other.y || x == other.x && y == other.y && z < other.z);
	}

    friend std::ostream& operator << (std::ostream& os, const VOXEL_LOC& p)
    {
        os  << "("<< p.x <<", "<<p.y<<", "<<p.z<<")"<<std::endl;
        return os;
    }
};

// Hash value
namespace std
{
	template <>
	struct hash<VOXEL_LOC>
	{
		int64_t operator()(const VOXEL_LOC &s) const
		{
			using std::hash;
			using std::size_t;
			return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
		}
	};
} // namespace std


struct OctreeMap : torch::CustomClassHolder
{
public:
	thuni::Octree octree_feature;
	float voxel_size = 0.1;

	OctreeMap()
	{
		octree_feature.set_min_extent(voxel_size/2);
		octree_feature.set_bucket_size(1);
		octree_feature.set_down_size();
	}

	~OctreeMap()
	{
		octree_feature.clear();
	}

	// 近邻搜索
	torch::Tensor nearest_search(torch::Tensor pts_attr, torch::Tensor params)
	{
		if(octree_feature.size()<1)
		{
			std::cout<<"Empty octree!\n";
			return torch::empty(0);;
		}
		int num = pts_attr.size(0);
		auto pts_attr_acces = pts_attr.accessor<float,2>();
		auto params_acces = params.accessor<float,1>();
		if(pts_attr.size(1)<3)
		{
			std::cout<<"Must be 3D pts!\n";
			return torch::empty(0);;
		}
		int attr_n = octree_feature.get_attr_n();
		torch::Tensor data_tensor = torch::zeros({num, attr_n}, torch::dtype(torch::kFloat32));
		auto data_tensor_acces = data_tensor.accessor<float,2>();
		for(int i=0; i<num; i++)
		{
			std::vector<float> query;
			for(int j=0; j<3; j++)
				query.push_back(pts_attr_acces[i][j]);
			std::vector<std::vector<float>> resultIndices;
			std::vector<float> distances;
			octree_feature.knnNeighbors_eigen(query, 1, resultIndices, distances);
			if(distances.size()>0)
			{
				for(int j=0; j<attr_n; j++)
				{
					data_tensor_acces[i][j] = resultIndices[0][j];
				}
			}
		}
		return data_tensor;
	}

	//添加点数据
	void add_pts_with_attr_cpu(torch::Tensor pts_attr, torch::Tensor params)
	{
		// params[0] min_depth
		// std::cout<<"pts_attr.type(): "<<pts_attr.type()<<std::endl; // CPUFloatType
		int num = pts_attr.size(0);
		const int attr_n = pts_attr.size(1)-3;
		auto pts_attr_acces = pts_attr.accessor<float,2>();
		// torch::Tensor params_cpu = params.to(torch::kCPU);
		auto params_acces = params.accessor<float,1>();
		std::cout<<"num: "<<num<<std::endl;
		std::cout<<"attr_n: "<<attr_n<<std::endl;
		if(attr_n<0)
		{
			std::cout<<"Must be 3D pts!\n";
			return;
		}
		std::vector<int> pc_ids_filtered;
		pc_voxel_filter(pts_attr_acces, num, pc_ids_filtered, voxel_size);
		num = pc_ids_filtered.size();
		std::cout<<"after filter num: "<<num<<std::endl;
		std::vector<std::vector<float>> pts(num), extra_attr(num);
		for(int i=0; i<num; i++)
		{
			for(int j=0; j<3; j++)
				pts[i].push_back(pts_attr_acces[pc_ids_filtered[i]][j]);
			for(int j=0; j<attr_n; j++)
				extra_attr[i].push_back(pts_attr_acces[pc_ids_filtered[i]][j+3]);
		}
		if(octree_feature.size()==0)
			octree_feature.initialize_with_attr(pts, extra_attr);
		else
			octree_feature.update_with_attr(pts, extra_attr);
		std::cout<<"octree_feature.get_size(): "<<octree_feature.get_size()<<std::endl;
	}

	torch::Tensor get_data()
	{
		std::vector<std::vector<float>> orig_data= octree_feature.get_orig_data();
		int num = orig_data.size();
		if(num<1)
		{
			return torch::empty(0);
		}
		int attr_n = orig_data[0].size();
		torch::Tensor data_tensor = torch::zeros({num, attr_n}, torch::dtype(torch::kFloat32));
		auto data_tensor_acces = data_tensor.accessor<float,2>();
		for(int i=0; i<num; i++)
		{
			for(int j=0; j<attr_n; j++)
			{
				data_tensor_acces[i][j] = orig_data[i][j];
			}
		}
		return data_tensor;
	}

	template <typename PCType = std::vector<std::vector<float>> >
    void pc_voxel_filter(const PCType & pc_in, int size, std::vector<int> & pc_ids_out, float voxel_size = 0.2, bool no_sort = false)
	{
		std::unordered_map<VOXEL_LOC, std::vector<int>> feat_map_tmp;
		for (int i=0; i<size; i++)
		{
			int64_t x = std::round(pc_in[i][0]/voxel_size);
			int64_t y = std::round(pc_in[i][1]/voxel_size);
			int64_t z = std::round(pc_in[i][2]/voxel_size);
			VOXEL_LOC position(x, y, z);
			feat_map_tmp[position].push_back(i);
		}
		pc_ids_out.clear();
		for (auto iter = feat_map_tmp.begin(); iter != feat_map_tmp.end(); ++iter)
		{
			if(no_sort)
			{
				pc_ids_out.push_back(iter->second[0]);
				continue;
			}
			int best_id = 0;
			float min_dist = 1e8;
			int pt_n = iter->second.size();
			Eigen::Vector3f center(iter->first.x, iter->first.y, iter->first.z);
			for(int i=0; i<pt_n; i++)
			{
				int id = iter->second[i];
				float dist = (center - Eigen::Vector3f(pc_in[id][0], pc_in[id][1], pc_in[id][2])).norm();
				if(dist<min_dist)
				{
					min_dist = dist;
					best_id = id;
				}
			}
			// pc_out.push_back(pc_in[best_id]);
			pc_ids_out.push_back(best_id);
		}
	}

	void debug_print()
	{
		std::cout<<"=============================="<<std::endl;
		std::cout<<"This is a debug print in OctreeMap C++!"<<std::endl;
		std::cout<<"=============================="<<std::endl;
	}

};

#endif


