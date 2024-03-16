#ifndef THUNI_OCTREE_H_
#define THUNI_OCTREE_H_

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

#include <stdint.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include <algorithm>
#include <utility>
#include <chrono>

namespace thuni
{

	template <typename T>
	struct PointType_CMP
	{
		T point;
		float dist = INFINITY;

		PointType_CMP(){};

		PointType_CMP(T p, float d)
		{
			this->point = p;
			this->dist = d;
		};

		bool operator<(const PointType_CMP &a) const
		{
			return dist < a.dist;
		}
	};

	// 自动排序堆栈！！首个元素永远是值最大的一个。对于kd树搜索，不需要完全排序，只要首个元素最大即可
	template <typename PointType>
	class MANUAL_HEAP
	{
	private:
		PointType_CMP<PointType> *heap;
		int heap_size = 0;
		int cap = 0;
	public:
		MANUAL_HEAP(int max_capacity = 100)
		{
			cap = max_capacity;
			heap = new PointType_CMP<PointType>[max_capacity];
			heap_size = 0;
		}

		~MANUAL_HEAP()
		{
			delete[] heap;
		}

		void pop()
		{
			if (heap_size == 0)
				return;
			heap[0] = heap[heap_size - 1];
			heap_size--;
			MoveDown(0);
			return;
		}

		PointType_CMP<PointType> top() { return heap[0]; }

		float top_v() { return heap[0].dist; }

		void push(PointType_CMP<PointType> point)
		{
			if (heap_size >= cap)
			{
				if (point < heap[0])
				{
					// std::cout<<"point.dist: "<<point.dist<<", heap[0].dist: "<<heap[0].dist<<std::endl;
					pop();
				}
				else
				{
					// std::cout<<"point.dist: "<<point.dist<<", heap[0].dist: "<<heap[0].dist<<std::endl;
					return;
				}
			}
			heap[heap_size] = point;
			FloatUp(heap_size);
			heap_size++;
			return;
		}

		bool full() { return heap_size >= cap; }

		int size() { return heap_size; }

		void clear()
		{
			heap_size = 0;
			return;
		}

		std::vector<PointType_CMP<PointType>> get_data()
		{
			std::vector<PointType_CMP<PointType>> datas;
			for (int i = 0; i < heap_size; i++)
			{
				datas.push_back(heap[i]);
			}
			return datas;
		}

	private:

		void MoveDown(int heap_index)
		{
			int l = heap_index * 2 + 1;
			PointType_CMP<PointType> tmp = heap[heap_index];
			while (l < heap_size)
			{
				if (l + 1 < heap_size && heap[l] < heap[l + 1])
					l++;
				if (tmp < heap[l])
				{
					heap[heap_index] = heap[l];
					heap_index = l;
					l = heap_index * 2 + 1;
				}
				else
					break;
			}
			heap[heap_index] = tmp;
			return;
		}

		void FloatUp(int heap_index)
		{
			int ancestor = (heap_index - 1) / 2;
			PointType_CMP<PointType> tmp = heap[heap_index]; // 新加入的数据
			while (heap_index > 0)
			{
				if (heap[ancestor] < tmp)
				{
					heap[heap_index] = heap[ancestor];
					heap_index = ancestor;
					ancestor = (heap_index - 1) / 2;
				}
				else
					break;
			}
			heap[heap_index] = tmp;
			return;
		}
	};
	
	struct DistanceIndex
	{
		float dist_;
		float* index_;

		DistanceIndex()
		{
			dist_ = std::numeric_limits<float>::max();
			index_ = nullptr;
		}

		DistanceIndex(float dist, float* index) :
			dist_(dist), index_(index)
		{
		}

		bool operator<(const DistanceIndex& dist_index) const
		{
			return dist_ < dist_index.dist_;
		}
	};

	class KNNSimpleResultSet
	{
	private:
		size_t capacity_;
		size_t count_;
		float worst_distance_;
		std::vector<DistanceIndex> dist_index_;
	public:
		KNNSimpleResultSet(size_t capacity_) :
			capacity_(capacity_)
		{
			// reserving capacity to prevent memory re-allocations
			dist_index_.resize(capacity_, DistanceIndex(std::numeric_limits<float>::max(), nullptr));
			clear();
		}
		const std::vector<DistanceIndex> & get_data()
		{
			return dist_index_;
		}

		~KNNSimpleResultSet()
		{
		}

		void clear()
		{
			worst_distance_ = std::numeric_limits<float>::max();
			dist_index_[capacity_-1].dist_ = worst_distance_;
			count_ = 0;
		}

		size_t size() const
		{
			return count_;
		}

		bool full() const
		{
			return count_==capacity_;
		}

		void addPoint(float dist, float* index)
		{
			if (dist>=worst_distance_) return;

			if (count_ < capacity_) ++count_;
			size_t i;
			for (i=count_-1; i>0; --i) 
			{
				if (dist_index_[i-1].dist_>dist)
					dist_index_[i] = dist_index_[i-1];
				else 
					break;
			}
			dist_index_[i].dist_ = dist;
			dist_index_[i].index_ = index;
			worst_distance_ = dist_index_[capacity_-1].dist_;
		}

		float worstDist() const
		{
			return worst_distance_;
		}
	};

	struct BoxDeleteType
	{
		float min[3];
		float max[3];
		void show()
		{
			printf("min:(%f, %f, %f), max:(%f, %f, %f)\n", min[0], min[1],min[2],max[0],max[1],max[2]);
		}
	};

	template <typename T=float >
	class Matrix
	{
	public:
		size_t rows;
		size_t cols;
		T* data;

		Matrix() : rows(0), cols(0), data(NULL)
		{
		}

		Matrix(T*  data_, size_t rows_, size_t cols_) :
			rows(rows_), cols(cols_)
		{
			data = data_;
		}

		void setdata(T*  data_, size_t rows_, size_t cols_)
		{
			rows = rows_;
			cols = cols_;
			data = data_;
		}

		inline T* operator[](size_t index) const
		{
			return data+index*cols;
		}

		T* ptr() const
		{
			return data;
		}

		void clear()
		{
			if(data)
				delete [] data;
			data = nullptr;
		}
	};

	struct RunDetails
	{
		int depth;     // 搜索深度
		int pts_n;     // 搜索的点数
		int node_n;    // 搜索的叶子节点数
		bool one_path; // 首次搜索的单条路径
		std::chrono::high_resolution_clock::time_point begin_;
		std::chrono::high_resolution_clock::time_point end_;
		float total_time;

		RunDetails()
		{
			clear();
		}

		void start()
		{
			begin_ = std::chrono::high_resolution_clock::now();
		}

		float end()
		{
			end_ = std::chrono::high_resolution_clock::now();
			float time = float(std::chrono::duration_cast<std::chrono::microseconds>(end_-begin_).count())/1e3;
			begin_ = end_;
			total_time+=time;
			return time;
		}

		void clear()
		{
			depth = pts_n = node_n = 0;
			one_path = true;
			total_time = 0.0f;
		}

		void show()
		{
			printf("octree\t depth: %d, node_n: %d, pts_n: %d\n\n",depth, node_n, pts_n);
		}

		friend std::ostream &operator<<(std::ostream &os, const RunDetails &c)
		{
			os << "octree depth: " << c.depth << ", node_n: " << c.node_n << ", pts_n: " << c.pts_n << std::endl;
			; // 以"a+bi"的形式输出
			return os;
		}
	};

	class Octant
	{
	public:
		bool isActive;
		float x, y, z;		 // center
		float extent;		 // half of side-length
		std::vector<float *> points;
		// Matrix<float> ordered_points;
		Octant **child; // 对于叶子节点可减少 56字节的内存需求

		Octant() : x(0.0f), y(0.0f), z(0.0f), extent(0.0f)
		{
			child = nullptr;
			isActive = true;
		}

		~Octant()
		{
			if (child != nullptr)
			{
				for (size_t i = 0; i < 8; ++i)
				{
					if(child[i] != 0)
						delete child[i];
				}
				delete child;
				child = nullptr;
			}
			else
			{
				delete [] points[0];
				std::vector<float *>().swap(points);
			}
		}

		size_t size()
		{
			size_t pts_num = 0;
			get_octant_size(this, pts_num);
			return pts_num;
		}

		void get_octant_size(Octant * octant, size_t & size_ )
		{
			if (octant->child == nullptr)
			{
				size_ += octant->points.size();
				return;
			}
			for (size_t c = 0; c < 8; ++c)
			{
				if (octant->child[c] == 0)
					continue;
				get_octant_size(octant->child[c], size_);
			}
		}

		void init_child()
		{
			child = new Octant *[8];
			memset(child, 0, 8 * sizeof(Octant *));
		}
	};

	class Octree
	{
	public:
		size_t m_bucketSize = 32;
		float m_minExtent = 0.01f;
		bool m_downSize = false;
		int dim=4;
		size_t ordered_indies[8][7]={
			{1, 2, 4, 3, 5, 6, 7},
			{0, 3, 5, 2, 4, 7, 6},
			{0, 3, 6, 1, 4, 7, 5},
			{1, 2, 7, 0, 5, 6, 4},
			{0, 5, 6, 1, 2, 7, 3},
			{1, 4, 7, 0, 3, 6, 2},
			{2, 4, 7, 0, 3, 5, 1},
			{3, 5, 6, 1, 2, 4, 0}
		};
		bool ordered = true;;
        RunDetails run_details;

		Octree()
			: m_bucketSize(32), m_minExtent(0.01f), m_root_(0), m_downSize(false)
		{
			pts_num_deleted = last_pts_num = 0;
		}

		Octree(size_t bucketSize_, bool copyPoints_, float minExtent_)
			: m_bucketSize(bucketSize_), m_minExtent(minExtent_), m_root_(0), m_downSize(false)
		{
			pts_num_deleted = last_pts_num =0;
		}

		Octree(size_t bucketSize_, bool copyPoints_, float minExtent_, int dim_)
			: m_bucketSize(bucketSize_), m_minExtent(minExtent_), m_root_(0), m_downSize(false)
		{
			pts_num_deleted = last_pts_num =0;
			if(dim_>4) dim = dim_;
		}

		~Octree(){
			clear();
		}

		size_t get_attr_n()
		{
			return dim-1;
		}

		void set_order(bool ordered_=false)
		{
			ordered = ordered_;
		}

		void set_min_extent(float extent) // 网格最小内接园半径
		{
			m_minExtent = extent;
		}

		void set_bucket_size(size_t bucket_size)
		{
			m_bucketSize = bucket_size;
		}

		void set_down_size(bool down_size = true)
		{
			m_downSize = down_size;
		}

		template <typename ContainerT>
		void initialize(ContainerT &pts_)
		{
			clear();
			const size_t pts_num = pts_.size();
			std::vector<float *> points;
			points.resize(pts_num, 0);
			size_t cloud_index = 0;
			float min[3]={0,0,0}, max[3]={0,0,0};
			for (size_t i = 0; i < pts_num; ++i)
			{
				const float & x = pts_[i].x;
				const float & y = pts_[i].y;
				const float & z = pts_[i].z;
				if (std::isnan(x) || std::isnan(y) || std::isnan(z))
					continue;
				float* cloud_ptr = new float[dim];
				cloud_ptr[0] = x;
				cloud_ptr[1] = y;
				cloud_ptr[2] = z;
				cloud_ptr[3] = float(cloud_index); // 保存在**原始**数据中的索引
				points[cloud_index] = cloud_ptr;
				if(cloud_index==0)
				{
					min[0] = max[0] = x;
					min[1] = max[1] = y;
					min[2] = max[2] = z;
				}
				else
				{
					min[0] = x<min[0] ? x : min[0];
					min[1] = y<min[1] ? y : min[1];
					min[2] = z<min[2] ? z : min[2];
					max[0] = x>max[0] ? x : max[0];
					max[1] = y>max[1] ? y : max[1];
					max[2] = z>max[2] ? z : max[2];
				}
				cloud_index++;
			}
			last_pts_num = cloud_index;
			points.resize(cloud_index); // 删除多余元素
			float ctr[3] = {min[0], min[1], min[2]};
			float maxextent = 0.5f * (max[0] - min[0]);
			ctr[0] += maxextent;
			for (size_t i = 1; i < 3; ++i)
			{
				float extent = 0.5f * (max[i] - min[i]);
				ctr[i] += extent;
				if (extent > maxextent)
					maxextent = extent;
			}
			// std::cout<<"maxextent: "<<maxextent<<", "
			// 		<<"min: "<<min[0]<<", "<<min[1]<<", "<<min[2]<<", "
			// 		<<"max: "<<max[0]<<", "<<max[1]<<", "<<max[2]<<", "
			// 		<<std::endl;
			// m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1, N);
			m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, points);
			// std::cout<<"createOctant success!"<<std::endl;
			for (size_t i = 0; i < points.size(); ++i)
			{
				delete [] points[i];
			}
		}

		template <typename ContainerT>
		void initialize_with_attr(ContainerT &pts_, std::vector<float> & extra_attr)
		{
			int extra_attr_num = extra_attr.size();
			dim += extra_attr_num;
			clear();
			const size_t pts_num = pts_.size();
			std::vector<float *> points;
			points.resize(pts_num, 0);
			size_t cloud_index = 0;
			float min[3]={0,0,0}, max[3]={0,0,0};
			for (size_t i = 0; i < pts_num; ++i)
			{
				// const float & x = pts_[i].x;
				// const float & y = pts_[i].y;
				// const float & z = pts_[i].z;
				const float & x = pts_[i][0];
				const float & y = pts_[i][1];
				const float & z = pts_[i][2];
				if (std::isnan(x) || std::isnan(y) || std::isnan(z))
					continue;
				float* cloud_ptr = new float[dim];
				cloud_ptr[0] = x;
				cloud_ptr[1] = y;
				cloud_ptr[2] = z;
				cloud_ptr[3] = float(cloud_index); // 保存在**原始**数据中的索引
				for(size_t j=0; j<extra_attr_num; j++)
				{
					cloud_ptr[4+j] = extra_attr[j];
				}
				points[cloud_index] = cloud_ptr;
				if(cloud_index==0)
				{
					min[0] = max[0] = x;
					min[1] = max[1] = y;
					min[2] = max[2] = z;
				}
				else
				{
					min[0] = x<min[0] ? x : min[0];
					min[1] = y<min[1] ? y : min[1];
					min[2] = z<min[2] ? z : min[2];
					max[0] = x>max[0] ? x : max[0];
					max[1] = y>max[1] ? y : max[1];
					max[2] = z>max[2] ? z : max[2];
				}
				cloud_index++;
			}
			last_pts_num = cloud_index;
			points.resize(cloud_index); // 删除多余元素
			float ctr[3] = {min[0], min[1], min[2]};
			float maxextent = 0.5f * (max[0] - min[0]);
			ctr[0] += maxextent;
			for (size_t i = 1; i < 3; ++i)
			{
				float extent = 0.5f * (max[i] - min[i]);
				ctr[i] += extent;
				if (extent > maxextent)
					maxextent = extent;
			}
			// std::cout<<"maxextent: "<<maxextent<<", "
			// 		<<"min: "<<min[0]<<", "<<min[1]<<", "<<min[2]<<", "
			// 		<<"max: "<<max[0]<<", "<<max[1]<<", "<<max[2]<<", "
			// 		<<std::endl;
			// m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1, N);
			m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, points);
			// std::cout<<"createOctant success!"<<std::endl;
			for (size_t i = 0; i < points.size(); ++i)
			{
				delete [] points[i];
			}
		}

		template <typename ContainerT>
		void update_with_attr(const ContainerT &pts_, std::vector<float> & extra_attr, bool down_size = false)
		{
			if (m_root_ == 0)
				return;
			int extra_attr_num = extra_attr.size();
			if(dim==4 && extra_attr_num>0) dim += extra_attr_num;
			// std::cout<<"update start\n";
			m_downSize = down_size;
			size_t pts_num = pts_.size();
			// std::cout<<"updateOctant init: "<<pts_num<<std::endl;
			std::vector<float *> points_tmp;
			points_tmp.resize(pts_num, 0);
			size_t cloud_index = 0;
			float min[3], max[3];
			const size_t N_old = last_pts_num;
			for (size_t i = 0; i < pts_num; ++i)
			{
				// const float & x = pts_[i].x;
				// const float & y = pts_[i].y;
				// const float & z = pts_[i].z;
				const float & x = pts_[i][0];
				const float & y = pts_[i][1];
				const float & z = pts_[i][2];
				if (std::isnan(x) || std::isnan(y) || std::isnan(z))
					continue;
				float* cloud_ptr = new float[dim];
				cloud_ptr[0] = x;
				cloud_ptr[1] = y;
				cloud_ptr[2] = z;
				cloud_ptr[3] = N_old + cloud_index;
				for(size_t j=0; j<extra_attr_num; j++)
				{
					cloud_ptr[4+j] = extra_attr[j];
				}
				points_tmp[cloud_index] = cloud_ptr;
				if(cloud_index==0)
				{
					min[0] = max[0] = x;
					min[1] = max[1] = y;
					min[2] = max[2] = z;
				}
				else
				{
					min[0] = x<min[0] ? x : min[0];
					min[1] = y<min[1] ? y : min[1];
					min[2] = z<min[2] ? z : min[2];
					max[0] = x>max[0] ? x : max[0];
					max[1] = y>max[1] ? y : max[1];
					max[2] = z>max[2] ? z : max[2];
				}
				cloud_index++;
			}
			if(cloud_index == 0)
				return;
			points_tmp.resize(cloud_index);
			// std::cout<<"updateOctant filter: "<<cloud_index<<std::endl;
			// 先创建一个对当前节点全包围的父节点，首先确定父节点中心所在的方向
			static const float factor[] = {-0.5f, 0.5f};
			// 判断是否存在越界
			while (std::abs(max[0] - m_root_->x) > m_root_->extent || std::abs(max[1] - m_root_->y) > m_root_->extent || std::abs(max[2] - m_root_->z) > m_root_->extent)
			{
				// 父节点中心坐标
				float parentExtent = 2 * m_root_->extent;
				float parentX = m_root_->x + factor[max[0] > m_root_->x] * parentExtent;
				float parentY = m_root_->y + factor[max[1] > m_root_->y] * parentExtent;
				float parentZ = m_root_->z + factor[max[2] > m_root_->z] * parentExtent;
				// 构造父节点
				Octant *octant = new Octant;
				octant->x = parentX;
				octant->y = parentY;
				octant->z = parentZ;
				octant->extent = parentExtent;
				octant->init_child();
				size_t mortonCode = 0;
				if (m_root_->x > parentX) mortonCode |= 1;
				if (m_root_->y > parentY) mortonCode |= 2;
				if (m_root_->z > parentZ) mortonCode |= 4;
				octant->child[mortonCode] = m_root_;
				m_root_ = octant;
			}
			while (std::abs(min[0] - m_root_->x) > m_root_->extent || std::abs(min[1] - m_root_->y) > m_root_->extent || std::abs(min[2] - m_root_->z) > m_root_->extent)
			{
				// 父节点中心坐标
				float parentExtent = 2 * m_root_->extent;
				float parentX = m_root_->x + factor[min[0] > m_root_->x] * parentExtent;
				float parentY = m_root_->y + factor[min[1] > m_root_->y] * parentExtent;
				float parentZ = m_root_->z + factor[min[2] > m_root_->z] * parentExtent;
				// 构造父节点
				Octant *octant = new Octant;
				// octant->isLeaf = false;
				octant->x = parentX;
				octant->y = parentY;
				octant->z = parentZ;
				octant->extent = parentExtent;
				octant->init_child();
				size_t mortonCode = 0;
				if (m_root_->x > parentX) mortonCode |= 1;
				if (m_root_->y > parentY) mortonCode |= 2;
				if (m_root_->z > parentZ) mortonCode |= 4;
				octant->child[mortonCode] = m_root_;
				m_root_ = octant;
			}

			if (points_tmp.size() == 0)
				return;
			// std::cout<<"updateOctant start: "<<points_tmp.size()<<std::endl;;
			last_pts_num += points_tmp.size();
			updateOctant(m_root_, points_tmp);
			// std::cout<<"updateOctant end\n";
			for (size_t i = 0; i < points_tmp.size(); ++i)
			{
				delete [] points_tmp[i];
			}
		}

		template <typename ContainerT>
		void initialize_with_attr(ContainerT &pts_, std::vector<std::vector<float>> & extra_attr)
		{
			if(pts_.size() != extra_attr.size())
			{
				printf("Error number of attr!\n");
				return;
			}
			int extra_attr_num = extra_attr[0].size();
			dim += extra_attr_num;
			clear();
			const size_t pts_num = pts_.size();
			std::vector<float *> points;
			points.resize(pts_num, 0);
			size_t cloud_index = 0;
			float min[3]={0,0,0}, max[3]={0,0,0};
			for (size_t i = 0; i < pts_num; ++i)
			{
				// const float & x = pts_[i].x;
				// const float & y = pts_[i].y;
				// const float & z = pts_[i].z;
				const float & x = pts_[i][0];
				const float & y = pts_[i][1];
				const float & z = pts_[i][2];
				if (std::isnan(x) || std::isnan(y) || std::isnan(z))
					continue;
				float* cloud_ptr = new float[dim];
				cloud_ptr[0] = x;
				cloud_ptr[1] = y;
				cloud_ptr[2] = z;
				cloud_ptr[3] = float(cloud_index); // 保存在**原始**数据中的索引
				for(size_t j=0; j<extra_attr_num; j++)
				{
					cloud_ptr[4+j] = extra_attr[i][j];
				}
				points[cloud_index] = cloud_ptr;
				if(cloud_index==0)
				{
					min[0] = max[0] = x;
					min[1] = max[1] = y;
					min[2] = max[2] = z;
				}
				else
				{
					min[0] = x<min[0] ? x : min[0];
					min[1] = y<min[1] ? y : min[1];
					min[2] = z<min[2] ? z : min[2];
					max[0] = x>max[0] ? x : max[0];
					max[1] = y>max[1] ? y : max[1];
					max[2] = z>max[2] ? z : max[2];
				}
				cloud_index++;
			}
			last_pts_num = cloud_index;
			points.resize(cloud_index); // 删除多余元素
			float ctr[3] = {min[0], min[1], min[2]};
			float maxextent = 0.5f * (max[0] - min[0]);
			ctr[0] += maxextent;
			for (size_t i = 1; i < 3; ++i)
			{
				float extent = 0.5f * (max[i] - min[i]);
				ctr[i] += extent;
				if (extent > maxextent)
					maxextent = extent;
			}
			// std::cout<<"maxextent: "<<maxextent<<", "
			// 		<<"min: "<<min[0]<<", "<<min[1]<<", "<<min[2]<<", "
			// 		<<"max: "<<max[0]<<", "<<max[1]<<", "<<max[2]<<", "
			// 		<<std::endl;
			// m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1, N);
			m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, points);
			// std::cout<<"createOctant success!"<<std::endl;
			for (size_t i = 0; i < points.size(); ++i)
			{
				delete [] points[i];
			}
		}

		template <typename ContainerT>
		void update_with_attr(const ContainerT &pts_, std::vector<std::vector<float>> & extra_attr, bool down_size = false)
		{
			if (m_root_ == 0)
				return;
			if(pts_.size() != extra_attr.size())
			{
				printf("Error number of attr!\n");
				return;
			}
			int extra_attr_num = extra_attr[0].size();
			if(dim==4 && extra_attr_num>0) dim += extra_attr_num;
			// std::cout<<"update start\n";
			m_downSize = down_size;
			size_t pts_num = pts_.size();
			// std::cout<<"updateOctant init: "<<pts_num<<std::endl;
			std::vector<float *> points_tmp;
			points_tmp.resize(pts_num, 0);
			size_t cloud_index = 0;
			float min[3], max[3];
			const size_t N_old = last_pts_num;
			for (size_t i = 0; i < pts_num; ++i)
			{
				// const float & x = pts_[i].x;
				// const float & y = pts_[i].y;
				// const float & z = pts_[i].z;
				const float & x = pts_[i][0];
				const float & y = pts_[i][1];
				const float & z = pts_[i][2];
				if (std::isnan(x) || std::isnan(y) || std::isnan(z))
					continue;
				float* cloud_ptr = new float[dim];
				cloud_ptr[0] = x;
				cloud_ptr[1] = y;
				cloud_ptr[2] = z;
				cloud_ptr[3] = N_old + cloud_index;
				for(size_t j=0; j<extra_attr_num; j++)
				{
					cloud_ptr[4+j] = extra_attr[i][j];
				}
				points_tmp[cloud_index] = cloud_ptr;
				if(cloud_index==0)
				{
					min[0] = max[0] = x;
					min[1] = max[1] = y;
					min[2] = max[2] = z;
				}
				else
				{
					min[0] = x<min[0] ? x : min[0];
					min[1] = y<min[1] ? y : min[1];
					min[2] = z<min[2] ? z : min[2];
					max[0] = x>max[0] ? x : max[0];
					max[1] = y>max[1] ? y : max[1];
					max[2] = z>max[2] ? z : max[2];
				}
				cloud_index++;
			}
			if(cloud_index == 0)
				return;
			points_tmp.resize(cloud_index);
			// std::cout<<"updateOctant filter: "<<cloud_index<<std::endl;
			// 先创建一个对当前节点全包围的父节点，首先确定父节点中心所在的方向
			static const float factor[] = {-0.5f, 0.5f};
			// 判断是否存在越界
			while (std::abs(max[0] - m_root_->x) > m_root_->extent || std::abs(max[1] - m_root_->y) > m_root_->extent || std::abs(max[2] - m_root_->z) > m_root_->extent)
			{
				// 父节点中心坐标
				float parentExtent = 2 * m_root_->extent;
				float parentX = m_root_->x + factor[max[0] > m_root_->x] * parentExtent;
				float parentY = m_root_->y + factor[max[1] > m_root_->y] * parentExtent;
				float parentZ = m_root_->z + factor[max[2] > m_root_->z] * parentExtent;
				// 构造父节点
				Octant *octant = new Octant;
				octant->x = parentX;
				octant->y = parentY;
				octant->z = parentZ;
				octant->extent = parentExtent;
				octant->init_child();
				size_t mortonCode = 0;
				if (m_root_->x > parentX) mortonCode |= 1;
				if (m_root_->y > parentY) mortonCode |= 2;
				if (m_root_->z > parentZ) mortonCode |= 4;
				octant->child[mortonCode] = m_root_;
				m_root_ = octant;
			}
			while (std::abs(min[0] - m_root_->x) > m_root_->extent || std::abs(min[1] - m_root_->y) > m_root_->extent || std::abs(min[2] - m_root_->z) > m_root_->extent)
			{
				// 父节点中心坐标
				float parentExtent = 2 * m_root_->extent;
				float parentX = m_root_->x + factor[min[0] > m_root_->x] * parentExtent;
				float parentY = m_root_->y + factor[min[1] > m_root_->y] * parentExtent;
				float parentZ = m_root_->z + factor[min[2] > m_root_->z] * parentExtent;
				// 构造父节点
				Octant *octant = new Octant;
				// octant->isLeaf = false;
				octant->x = parentX;
				octant->y = parentY;
				octant->z = parentZ;
				octant->extent = parentExtent;
				octant->init_child();
				size_t mortonCode = 0;
				if (m_root_->x > parentX) mortonCode |= 1;
				if (m_root_->y > parentY) mortonCode |= 2;
				if (m_root_->z > parentZ) mortonCode |= 4;
				octant->child[mortonCode] = m_root_;
				m_root_ = octant;
			}

			if (points_tmp.size() == 0)
				return;
			// std::cout<<"updateOctant start: "<<points_tmp.size()<<std::endl;;
			last_pts_num += points_tmp.size();
			updateOctant(m_root_, points_tmp);
			// std::cout<<"updateOctant end\n";
			for (size_t i = 0; i < points_tmp.size(); ++i)
			{
				delete [] points_tmp[i];
			}
		}

		template <typename ContainerT>
		void update(const ContainerT &pts_, bool down_size = false)
		{
			if (m_root_ == 0)
				return;
			// std::cout<<"update start\n";
			m_downSize = down_size;
			size_t pts_num = pts_.size();
			// std::cout<<"updateOctant init: "<<pts_num<<std::endl;
			std::vector<float *> points_tmp;
			points_tmp.resize(pts_num, 0);
			size_t cloud_index = 0;
			float min[3], max[3];
			const size_t N_old = last_pts_num;
			for (size_t i = 0; i < pts_num; ++i)
			{
				const float & x = pts_[i].x;
				const float & y = pts_[i].y;
				const float & z = pts_[i].z;
				if (std::isnan(x) || std::isnan(y) || std::isnan(z))
					continue;
				float* cloud_ptr = new float[dim];
				cloud_ptr[0] = x;
				cloud_ptr[1] = y;
				cloud_ptr[2] = z;
				cloud_ptr[3] = N_old + cloud_index;
				points_tmp[cloud_index] = cloud_ptr;
				if(cloud_index==0)
				{
					min[0] = max[0] = x;
					min[1] = max[1] = y;
					min[2] = max[2] = z;
				}
				else
				{
					min[0] = x<min[0] ? x : min[0];
					min[1] = y<min[1] ? y : min[1];
					min[2] = z<min[2] ? z : min[2];
					max[0] = x>max[0] ? x : max[0];
					max[1] = y>max[1] ? y : max[1];
					max[2] = z>max[2] ? z : max[2];
				}
				cloud_index++;
			}
			if(cloud_index == 0)
				return;
			points_tmp.resize(cloud_index);
			// std::cout<<"updateOctant filter: "<<cloud_index<<std::endl;
			// 先创建一个对当前节点全包围的父节点，首先确定父节点中心所在的方向
			static const float factor[] = {-0.5f, 0.5f};
			// 判断是否存在越界
			while (std::abs(max[0] - m_root_->x) > m_root_->extent || std::abs(max[1] - m_root_->y) > m_root_->extent || std::abs(max[2] - m_root_->z) > m_root_->extent)
			{
				// 父节点中心坐标
				float parentExtent = 2 * m_root_->extent;
				float parentX = m_root_->x + factor[max[0] > m_root_->x] * parentExtent;
				float parentY = m_root_->y + factor[max[1] > m_root_->y] * parentExtent;
				float parentZ = m_root_->z + factor[max[2] > m_root_->z] * parentExtent;
				// 构造父节点
				Octant *octant = new Octant;
				octant->x = parentX;
				octant->y = parentY;
				octant->z = parentZ;
				octant->extent = parentExtent;
				octant->init_child();
				size_t mortonCode = 0;
				if (m_root_->x > parentX) mortonCode |= 1;
				if (m_root_->y > parentY) mortonCode |= 2;
				if (m_root_->z > parentZ) mortonCode |= 4;
				octant->child[mortonCode] = m_root_;
				m_root_ = octant;
			}
			while (std::abs(min[0] - m_root_->x) > m_root_->extent || std::abs(min[1] - m_root_->y) > m_root_->extent || std::abs(min[2] - m_root_->z) > m_root_->extent)
			{
				// 父节点中心坐标
				float parentExtent = 2 * m_root_->extent;
				float parentX = m_root_->x + factor[min[0] > m_root_->x] * parentExtent;
				float parentY = m_root_->y + factor[min[1] > m_root_->y] * parentExtent;
				float parentZ = m_root_->z + factor[min[2] > m_root_->z] * parentExtent;
				// 构造父节点
				Octant *octant = new Octant;
				// octant->isLeaf = false;
				octant->x = parentX;
				octant->y = parentY;
				octant->z = parentZ;
				octant->extent = parentExtent;
				octant->init_child();
				size_t mortonCode = 0;
				if (m_root_->x > parentX) mortonCode |= 1;
				if (m_root_->y > parentY) mortonCode |= 2;
				if (m_root_->z > parentZ) mortonCode |= 4;
				octant->child[mortonCode] = m_root_;
				m_root_ = octant;
			}

			if (points_tmp.size() == 0)
				return;
			// std::cout<<"updateOctant start: "<<points_tmp.size()<<std::endl;;
			last_pts_num += points_tmp.size();
			updateOctant(m_root_, points_tmp);
			// std::cout<<"updateOctant end\n";
			for (size_t i = 0; i < points_tmp.size(); ++i)
			{
				delete [] points_tmp[i];
			}
		}

		void clear()
		{
			delete m_root_;
			m_root_ = 0;
			pts_num_deleted = last_pts_num =0;
		}

		template <typename PointT>
		void radiusNeighbors(const PointT &  query, float radius, std::vector<size_t> &resultIndices)
		{
			resultIndices.clear();
			if (m_root_ == 0)
				return;
			float sqrRadius = radius*radius; // "squared" radius
			float query_[3] = {query.x, query.y, query.z};
			std::vector<float*> points_ptr;
			radiusNeighbors(m_root_, query_, radius, sqrRadius, points_ptr);
			resultIndices.resize(points_ptr.size());
			for (size_t i=0;i<points_ptr.size();i++)
			{
				resultIndices[i] = size_t(points_ptr[i][3]);
			}
		}

		template <typename PointT>
		void radiusNeighbors(const PointT &  query, float radius, std::vector<size_t> &resultIndices, std::vector<float> &distances)
		{
			resultIndices.clear();
			distances.clear();
			if (m_root_ == 0)
				return;
			float sqrRadius = radius*radius; // "squared" radius
			float query_[3] = {query.x, query.y, query.z};
			std::vector<float*> points_ptr;
			radiusNeighbors(m_root_, query_, radius, sqrRadius, points_ptr, distances);
			// radiusNeighbors2(m_root_, query_, sqrRadius, resultIndices, distances);
			resultIndices.resize(points_ptr.size());
			for (size_t i=0;i<points_ptr.size();i++)
			{
				resultIndices[i] = size_t(points_ptr[i][3]);
			}
		}

		template <typename PointT>
		void radiusNeighbors(const PointT &  query, float radius, std::vector<PointT> &resultIndices, std::vector<float> &distances)
		{
			resultIndices.clear();
			distances.clear();
			if (m_root_ == 0)
				return;
			float sqrRadius = radius*radius; // "squared" radius
			float query_[3] = {query.x, query.y, query.z};
			std::vector<float*> points_ptr;
			radiusNeighbors(m_root_, query_, radius, sqrRadius, points_ptr, distances);
			// radiusNeighbors2(m_root_, query_, sqrRadius, resultIndices, distances);
			resultIndices.resize(points_ptr.size());
			for (size_t i=0;i<resultIndices.size();i++)
			{
				PointT pt;
				pt.x = points_ptr[i][0];
				pt.y = points_ptr[i][1];
				pt.z = points_ptr[i][2];
				resultIndices[i] = pt;
			}
		}

		template <typename PointT>
		int32_t knnNeighbors(const PointT &  query, int k, std::vector<PointT> &resultIndices, std::vector<float> &distances)
		{
			if (m_root_ == 0)
				return 0;
			// MANUAL_HEAP<size_t> heap(k);
			// knnNeighbors(m_root_, query, heap);
			// std::cout<<"knnNeighbors start"<<std::endl;
			float query_[3] = {query.x, query.y, query.z};
			// run_details.clear();
			// run_details.start();
			KNNSimpleResultSet heap(k);
			knnNeighbors(m_root_, query_, heap);
			// run_details.end();
			// run_details.show();
			std::vector<DistanceIndex>  data = heap.get_data();
			resultIndices.resize(heap.size());
			distances.resize(heap.size());
			for (size_t i=0;i<heap.size();i++)
			{
				PointT pt;
				pt.x = data[i].index_[0];
				pt.y = data[i].index_[1];
				pt.z = data[i].index_[2];
				resultIndices[i] = pt;
				distances[i] = data[i].dist_;
			}
			// run_details.end();
			// run_details.show();
			return data.size();
		}

		template <typename EigenPointT>
		int32_t knnNeighbors_eigen(const EigenPointT &  query, int k, std::vector<std::vector<float>> &resultIndices, std::vector<float> &distances)
		{
			if (m_root_ == 0)
				return 0;
			float query_[3] = {(float)query[0], (float)query[1], (float)query[2]};
			KNNSimpleResultSet heap(k);
			knnNeighbors(m_root_, query_, heap);
			std::vector<DistanceIndex>  data = heap.get_data();
			resultIndices.resize(heap.size());
			distances.resize(heap.size());
			for (size_t i=0;i<heap.size();i++)
			{
				std::vector<float> pt;
				for(int j=0; j<dim; j++)
				{
					if(j==3) continue;
					pt.push_back(data[i].index_[j]);
				}
				resultIndices[i] = pt;
				distances[i] = data[i].dist_;
			}
			return data.size();
		}

		template <typename PointT>
		int32_t knnNeighbors(const PointT &  query, int k, std::vector<size_t> &resultIndices, std::vector<float> &distances)
		{
			if (m_root_ == 0)
				return 0;
			// MANUAL_HEAP<size_t> heap(k);
			// knnNeighbors(m_root_, query, heap);
			// std::cout<<"knnNeighbors start"<<std::endl;
			float query_[3] = {query.x, query.y, query.z};
			// run_details.clear();
			// run_details.start();
			KNNSimpleResultSet heap(k);
			knnNeighbors(m_root_, query_, heap);
			// run_details.end();
			// run_details.show();
			std::vector<DistanceIndex>  data = heap.get_data();
			resultIndices.resize(heap.size());
			distances.resize(heap.size());
			for (int i=0;i<heap.size();i++)
			{
				resultIndices[i] = size_t(data[i].index_[3]);
				distances[i] = data[i].dist_;
			}
			// run_details.end();
			// run_details.show();
			return data.size();
		}

		template <typename PointT>
		int32_t knnNeighbors(const PointT &  query, int k, std::vector<int> &resultIndices, std::vector<float> &distances)
		{
			if (m_root_ == 0)
				return 0;
			// MANUAL_HEAP<size_t> heap(k);
			// knnNeighbors(m_root_, query, heap);
			// std::cout<<"knnNeighbors start"<<std::endl;
			float query_[3] = {query.x, query.y, query.z};
			// run_details.clear();
			// run_details.start();
			KNNSimpleResultSet heap(k);
			knnNeighbors(m_root_, query_, heap);
			// run_details.end();
			// run_details.show();
			std::vector<DistanceIndex>  data = heap.get_data();
			resultIndices.resize(heap.size());
			distances.resize(heap.size());
			for (int i=0;i<heap.size();i++)
			{
				resultIndices[i] = int(data[i].index_[3]);
				distances[i] = data[i].dist_;
			}
			// run_details.end();
			// run_details.show();
			return data.size();
		}

		void boxWiseDelete(const BoxDeleteType & box_range, bool clear_data)
		{
			if (m_root_ == 0)
				return;
			bool deleted = false;
			boxWiseDelete(m_root_, box_range, deleted, clear_data);
			if(deleted) m_root_=0;
		}

		void boxWiseDelete(const float * min, const float * max, bool clear_data)
		{
			if (m_root_ == 0)
				return;
			BoxDeleteType box_range;
			box_range.min[0] = min[0];
			box_range.min[1] = min[1];
			box_range.min[2] = min[2];
			box_range.max[0] = max[0];
			box_range.max[1] = max[1];
			box_range.max[2] = max[2];
			// box_range.show();
			// printf("clear_data:%d \n", clear_data);
			bool deleted = false;
			boxWiseDelete(m_root_, box_range, deleted, clear_data);
			if(deleted) m_root_=0;
		}

		size_t size()
		{
			return last_pts_num -pts_num_deleted;
		}

		void get_nodes(Octant *octant, std::vector<Octant *> &nodes, float min_extent = 0)
		{
			if (octant == 0)
				return;
			if (min_extent > 0)
			{
				if (octant->extent <= min_extent)
				{
					nodes.push_back(octant);
					return;
				}
			}
			if (octant->child == nullptr)
			{
				nodes.push_back(octant);
				return;
			}
			for (int i = 0; i < 8; i++)
			{
				get_nodes(octant->child[i], nodes, min_extent);
			}
		}

		void get_leaf_nodes(const Octant *octant, std::vector<const Octant *> &nodes) const
		{
			if (octant == 0)
				return;
			if (octant->child == nullptr)
			{
				nodes.push_back(octant);
				return;
			}
			nodes.reserve(nodes.size()+8);
			for (int i = 0; i < 8; i++)
			{
				get_leaf_nodes(octant->child[i], nodes);
			}
		}

		template <typename PointT, typename ContainerT>
		ContainerT get_data()
		{
			std::vector<Octant *> nodes;
			ContainerT pts;
			get_nodes(m_root_, nodes);
			for(auto octant : nodes)
			{
				for(auto p: octant->points)
				{
					PointT pt;
					pt.x = p[0];
					pt.y = p[1];
					pt.z = p[2];
					pts.push_back(pt);
				}
			}
			return pts;
		}

		std::vector<std::vector<float>> get_orig_data()
		{
			std::vector<Octant *> nodes;
			std::vector<std::vector<float>> result;
			get_nodes(m_root_, nodes);
			for(auto octant : nodes)
			{
				for(auto p: octant->points)
				{
					std::vector<float> pt;
					for(int i=0; i<dim; i++)
					{
						if(i==3) continue;
						pt.push_back(p[i]);
					}
					result.push_back(pt);
				}
			}
			return result;
		}

		size_t get_size()
		{
			size_t actual_size = 0;
			std::vector<Octant *> nodes;
			get_nodes(m_root_, nodes);
			for(auto octant : nodes)
			{
				actual_size += octant->points.size();
			}
			return actual_size;
		}

	protected:
		Octant *m_root_;
		size_t last_pts_num, pts_num_deleted; // 主要为了确定每个点的索引

		Octree(Octree &);

		Octree &operator=(const Octree &oct);

		Octant *createOctant(float x, float y, float z, float extent, std::vector<float *> & points)
		{
			// For a leaf we don't have to change anything; points are already correctly linked or correctly reordered.
			Octant *octant = new Octant;
			const size_t size = points.size();
			octant->x = x;
			octant->y = y;
			octant->z = z;
			octant->extent = extent;
			static const float factor[] = {-0.5f, 0.5f};
			if (size > m_bucketSize && extent > 2 * m_minExtent) // 32 0
			{
				std::vector<std::vector<float *>> child_points(8, std::vector<float *>());
				for (size_t i = 0; i < size; ++i)
				{
					float * p = points[i];
					size_t mortonCode = 0;
					if (p[0] > x) mortonCode |= 1;
					if (p[1] > y) mortonCode |= 2;
					if (p[2] > z) mortonCode |= 4;
					child_points[mortonCode].push_back(p);
				}
				// now, we can create the child nodes...
				float childExtent = 0.5f * extent;
				octant->init_child();
				for (size_t i = 0; i < 8; ++i)
				{
					if (child_points[i].size() == 0)
						continue;
					float childX = x + factor[(i & 1) > 0] * extent;
					float childY = y + factor[(i & 2) > 0] * extent;
					float childZ = z + factor[(i & 4) > 0] * extent;
					octant->child[i] = createOctant(childX, childY, childZ, childExtent, child_points[i]);
				}
			}
			else
			{
				size_t size = points.size();
				if (m_downSize && extent <= 2 * m_minExtent && size > m_bucketSize / 8)
					size = m_bucketSize;
				octant->points.resize(size, 0);
				float * continue_points = new float[size * dim];
				for (size_t i = 0; i < size; ++i)
				{
					std::copy(points[i], points[i] + dim, continue_points+dim*i);
					octant->points[i] = continue_points+dim*i;
				}
			}
			return octant;
		}

		void updateOctant(Octant *octant, const std::vector<float *> & points)
		{
			// std::cout<<"updateOctant0 start "<<points.size()<<std::endl;
			static const float factor[] = {-0.5f, 0.5f};
			const float x = octant->x, y = octant->y, z = octant->z, extent = octant->extent;
			octant->isActive = true; // 更新状态
			if (octant->child == nullptr)
			{
				if (octant->points.size()+points.size() > m_bucketSize && extent > 2 * m_minExtent) // 32 0
				{
					octant->points.insert(octant->points.end(), points.begin(), points.end());
					const size_t size = octant->points.size();
					std::vector<std::vector<float *>> child_points(8, std::vector<float *>());
					for (size_t i = 0; i < size; ++i)
					{
						size_t mortonCode = 0;
						if (octant->points[i][0] > x) mortonCode |= 1;
						if (octant->points[i][1] > y) mortonCode |= 2;
						if (octant->points[i][2] > z) mortonCode |= 4;
						child_points[mortonCode].push_back(octant->points[i]);
					}
					float childExtent = 0.5f * extent;
					octant->init_child();
					for (size_t i = 0; i < 8; ++i)
					{
						if (child_points[i].size() == 0)
							continue;
						float childX = x + factor[(i & 1) > 0] * extent;
						float childY = y + factor[(i & 2) > 0] * extent;
						float childZ = z + factor[(i & 4) > 0] * extent;
						octant->child[i] = createOctant(childX, childY, childZ, childExtent, child_points[i]);
					}
					delete [] octant->points[0];
					std::vector<float *>().swap(octant->points); // 清空非叶子节点索引
				}
				else
				{
					//* 如果有下采样且满足条件，直接不添加
					if (m_downSize && extent <= 2 * m_minExtent && octant->points.size() > m_bucketSize / 8)
						return;
					octant->points.insert(octant->points.end(), points.begin(), points.end());
					const size_t size = octant->points.size();
					float * continue_points = new float[size * dim];
					float * old_points = octant->points[0];
					for (size_t i = 0; i < size; ++i)
					{
						std::copy(octant->points[i], octant->points[i] + dim, continue_points+dim*i);
						octant->points[i] = continue_points+dim*i;
					}
					delete [] old_points;
				}
			}
			else
			{
				const size_t size = points.size();
				std::vector<std::vector<float *>> child_points(8, std::vector<float *>());
				for (size_t i = 0; i < size; ++i)
				{
					size_t mortonCode = 0;
					if (points[i][0] > x) mortonCode |= 1;
					if (points[i][1] > y) mortonCode |= 2;
					if (points[i][2] > z) mortonCode |= 4;
					child_points[mortonCode].push_back(points[i]);
				}
				float childExtent = 0.5f * extent;
				for (size_t i = 0; i < 8; ++i)
				{
					if (child_points[i].size() > 0) // 可能存在某些节点没有新分配点，但是存在点的情况！！！
					{
						if (octant->child[i] == 0)
						{
							float childX = x + factor[(i & 1) > 0] * extent;
							float childY = y + factor[(i & 2) > 0] * extent;
							float childZ = z + factor[(i & 4) > 0] * extent;
							octant->child[i] = createOctant(childX, childY, childZ, childExtent, child_points[i]);
						}
						else
							updateOctant(octant->child[i], child_points[i]);
					}
				}
			}
		}

		void radiusNeighbors(const Octant *octant, const float * query, float radius, float sqrRadius, std::vector<float*> &resultIndices)
		{
			if (!octant->isActive)
				return;
			if (3*octant->extent*octant->extent<sqrRadius && contains(query, sqrRadius, octant))
			{
				// printf("contains\n");
				std::vector<const Octant *> candidate_octants;
				candidate_octants.reserve(8);
				get_leaf_nodes(octant, candidate_octants);
				for(size_t k=0; k<candidate_octants.size(); k++)
				{
					const size_t size = candidate_octants[k]->points.size();
					const size_t result_size = resultIndices.size();
					resultIndices.resize(result_size+size);
					for (size_t i = 0; i < size; ++i)
					{
						// const float * p = ordered? candidate_octants[k]->ordered_points[i] : candidate_octants[k]->points[i];
						// const float * p = candidate_octants[k]->points[i];
						resultIndices[result_size+i] = candidate_octants[k]->points[i];
					}
				}
				return;
			}
			if (octant->child == nullptr)
			{
				const size_t size = octant->points.size();
				for (size_t i = 0; i < size; ++i)
				{
					// const float * p = ordered? octant->ordered_points[i] : octant->points[i];
					const float * p = octant->points[i];
					float dist = 0, diff = 0;
					for(size_t j = 0; j < 3; ++j )
					{
						diff = p[j] - query[j];
						dist += diff*diff;
					}
					if (dist < sqrRadius)
						resultIndices.push_back(octant->points[i]);
				}
				return;
			}
			for (size_t c = 0; c < 8; ++c)
			{
				if (octant->child[c] == 0)
					continue;
				if (!overlaps(query, sqrRadius, octant->child[c]))
					continue;
				radiusNeighbors(octant->child[c], query, radius, sqrRadius, resultIndices);
			}
		}

		void radiusNeighbors(const Octant *octant, const float * query, float radius, float sqrRadius, std::vector<float*> &resultIndices, std::vector<float> &distances)
		{
			if (!octant->isActive)
				return;
			if (3*octant->extent*octant->extent<sqrRadius && contains(query, sqrRadius, octant))
			{
				std::vector<const Octant *> candidate_octants;
				get_leaf_nodes(octant, candidate_octants);
				for(size_t k=0; k<candidate_octants.size(); k++)
				{
					const size_t size = candidate_octants[k]->points.size();
					const size_t result_size = resultIndices.size();
					resultIndices.resize(result_size+size);
					for (size_t i = 0; i < size; ++i)
					{
						const float * p = candidate_octants[k]->points[i];
						float dist = 0, diff = 0;
						for(size_t j = 0; j < 3; ++j )
						{
							diff = p[j] - query[j];
							dist += diff*diff;
						}
						distances.push_back(dist);
						resultIndices[result_size+i] = candidate_octants[k]->points[i];
					}
				}
				return;
			}
			if (octant->child == nullptr)
			{
				const size_t size = octant->points.size();
				for (size_t i = 0; i < size; ++i)
				{
					const float * p = octant->points[i];
					float dist = 0, diff = 0;
					for(size_t j = 0; j < 3; ++j )
					{
						diff = p[j] - query[j];
						dist += diff*diff;
					}
					if (dist < sqrRadius)
					{
						resultIndices.push_back(octant->points[i]);
						distances.push_back(dist);
					}
				}
				return;
			}
			for (size_t c = 0; c < 8; ++c)
			{
				if (octant->child[c] == 0)
					continue;
				if (!overlaps(query, sqrRadius, octant->child[c]))
					continue;
				radiusNeighbors(octant->child[c], query, radius, sqrRadius, resultIndices, distances);
			}
		}

		bool radiusNeighbors2(Octant *octant, const float * query, float sqrRadius, std::vector<size_t> &resultIndices, std::vector<float> &distances)
		{
			if (!octant->isActive)
				return false;
			if (octant->child == nullptr)
			{
				const size_t size = octant->points.size();
				for (size_t i = 0; i < size; ++i)
				{
					// const float * p = ordered? octant->ordered_points[i] : octant->points[i];
					const float * p = octant->points[i];
					float dist = 0, diff = 0;
					for(size_t j = 0; j < 3; ++j )
					{
						diff = p[j] - query[j];
						dist += diff*diff;
					}
					if(dist<sqrRadius)
					{
						resultIndices.push_back(size_t(p[3]));
						distances.push_back(dist);
					}
				}
				return inside(query, sqrRadius, octant); // 如果堆已经满了且最远点在当前网格内，则不必搜索了
			}
			size_t mortonCode = 0;
			if (query[0] > octant->x) mortonCode |= 1;
			if (query[1] > octant->y) mortonCode |= 2;
			if (query[2] > octant->z) mortonCode |= 4;
			if (octant->child[mortonCode] != 0)
			{
				if (radiusNeighbors2(octant->child[mortonCode], query, sqrRadius, resultIndices, distances))
					return true;
			}
			for (int i = 0; i < 7; ++i)
			{
				int c = ordered_indies[mortonCode][i];
				if (octant->child[c] == 0)
					continue;
				if (!overlaps(query, sqrRadius, octant->child[c]))
					continue;
				if (radiusNeighbors2(octant->child[c], query, sqrRadius, resultIndices, distances))
					return true;
			}
			return inside(query, sqrRadius, octant);
		}

		bool knnNeighbors(Octant *octant, const float * query, KNNSimpleResultSet &heap)
		{
			// if (run_details.one_path) run_details.depth++;
			if (!octant->isActive)
				return false;
			if (octant->child == nullptr)
			{
				const size_t size = octant->points.size();
				for (size_t i = 0; i < size; ++i)
				{
					// const float * p = ordered? octant->ordered_points[i] : octant->points[i];
					const float * p = octant->points[i];
					float dist = 0, diff = 0;
					for(size_t j = 0; j < 3; ++j )
					{
						diff = p[j] - query[j];
						dist += diff*diff;
					}
					if(dist<heap.worstDist())
						heap.addPoint(dist, octant->points[i]);
						// heap.addPoint(dist, size_t(p[3]));
				}
				// run_details.one_path = false;
				// run_details.pts_n += size;
				// run_details.node_n++;
				// run_details.show();
				return heap.full() && inside(query, heap.worstDist(), octant); // 如果堆已经满了且最远点在当前网格内，则不必搜索了
			}
			size_t mortonCode = 0;
			if (query[0] > octant->x) mortonCode |= 1;
			if (query[1] > octant->y) mortonCode |= 2;
			if (query[2] > octant->z) mortonCode |= 4;
			if (octant->child[mortonCode] != 0)
			{
				if (knnNeighbors(octant->child[mortonCode], query, heap))
					return true;
			}
			for (int i = 0; i < 7; ++i)
			{
				int c = ordered_indies[mortonCode][i];
				if (octant->child[c] == 0)
					continue;
				if (heap.full() && !overlaps(query, heap.worstDist(), octant->child[c]))
					continue;
				if (knnNeighbors(octant->child[c], query, heap))
					return true;
			}
			return heap.full() && inside(query, heap.worstDist(), octant);
		}

		bool knnNeighbors(Octant *octant, const float * query, MANUAL_HEAP<size_t> &heap)
		{
			// 采用优先级队列，队列里的octant相互独立，距离检索点比较近的octant优先被搜索，重新写一个搜索算法，之后方便作为对比
			if (!octant->isActive)
				return false;
			if (octant->child == nullptr)
			{
				const size_t size = octant->points.size();
				for (size_t i = 0; i < size; ++i)
				{
					// const float * p = ordered? octant->ordered_points[i] : octant->points[i];
					const float * p = octant->points[i];
					float dist = 0, diff = 0;
					for(size_t j = 0; j < 3; ++j ) {
						diff = *p++ - *query++;
						dist += diff*diff;
					}
					PointType_CMP<size_t> pt(size_t(p[3]), dist);
					heap.push(pt);
				}
				return heap.full() && inside(query, heap.top_v(), octant); // 如果堆已经满了且最远点在当前网格内，则不必搜索了
			}
			size_t mortonCode = 0;
			if (query[0] > octant->x) mortonCode |= 1;
			if (query[1] > octant->y) mortonCode |= 2;
			if (query[2] > octant->z) mortonCode |= 4;
			if (octant->child != nullptr && octant->child[mortonCode] != 0)
			{
				if (knnNeighbors(octant->child[mortonCode], query, heap))
					return true;
			}
			for (size_t c = 0; c < 8 && octant->child != nullptr; ++c)
			{
				if (c == mortonCode)
					continue;
				if (octant->child[c] == 0)
					continue;
				if (heap.full() && !overlaps(query, heap.top_v(), octant->child[c]))
					continue;
				if (knnNeighbors(octant->child[c], query, heap))
					return true;
			}
			return heap.full() && inside(query, heap.top_v(), octant);
		}

		void boxWiseDelete(Octant *octant, const BoxDeleteType & box_range, bool & deleted, bool clear_data)
		{
			float cur_min[3];
			float cur_max[3];
			cur_min[0] = octant->x - octant->extent;
			cur_min[1] = octant->y - octant->extent;
			cur_min[2] = octant->z - octant->extent;
			cur_max[0] = octant->x + octant->extent;
			cur_max[1] = octant->y + octant->extent;
			cur_max[2] = octant->z + octant->extent;
			if (cur_min[0] > box_range.max[0] || box_range.min[0] > cur_max[0])
				return;
			if (cur_min[1] > box_range.max[1] || box_range.min[1] > cur_max[1])
				return;
			if (cur_min[2] > box_range.max[2] || box_range.min[2] > cur_max[2])
				return;
			// printf("octant->extent: %f\n", octant->extent);
			// printf("octant->extent: %f, %d \n", octant->extent,clear_data);
			// if(!clear_data) exit(1);
			// 确定有交集
			if (cur_min[0] >= box_range.min[0] && cur_min[1] >= box_range.min[1] && cur_min[2] >= box_range.min[2] &&
				cur_max[0] <= box_range.max[0] && cur_max[1] <= box_range.max[1] && cur_max[2] <= box_range.max[2])
			{
				if(!clear_data)
				{
					octant->isActive = false;
					return;
				}
				else
				{
					pts_num_deleted += octant->size();
					delete octant;
					deleted = true;
					return;
				}
			}
			if (octant->child == nullptr)
			{
				// printf("octant->extent: %f, %d \n", octant->extent,clear_data);
				if(!clear_data)
				{
					octant->isActive = false;
					return;
				}
				else
				{
					// printf("octant->extent: %f\n", octant->extent);
					const size_t size = octant->points.size();
					std::vector<float *> remainder_points;
					remainder_points.resize(size, 0);
					size_t valid_num = 0;
					for (size_t i = 0; i < size; ++i)
					{
						const float * p = octant->points[i];
						if (p[0] > box_range.max[0] || box_range.min[0] > p[0])
						{
							remainder_points[valid_num] = octant->points[i];
							valid_num++;
							continue;
						}
						if (p[1] > box_range.max[1] || box_range.min[1] > p[1])
						{
							remainder_points[valid_num] = octant->points[i];
							valid_num++;
							continue;
						}
						if (p[2] > box_range.max[2] || box_range.min[2] > p[2])
						{
							remainder_points[valid_num] = octant->points[i];
							valid_num++;
							continue;
						}
					}
					// printf("valid_num: %d\n", valid_num);
					pts_num_deleted += size-valid_num;
					if(valid_num==0)
					{
						delete octant;
						deleted = true;
						return;
					}
					float * continue_points = new float[valid_num * dim];
					float * old_points = octant->points[0];
					for (size_t i = 0; i < valid_num; ++i)
					{
						std::copy(remainder_points[i],remainder_points[i] + dim, continue_points+dim*i);
						octant->points[i] = continue_points+dim*i;
					}
					octant->points.resize(valid_num);
					delete [] old_points;
					// printf("delete: \n");
					return;
				}
			}
			
			// check whether child nodes are in range.
			for (size_t c = 0; c < 8; ++c)
			{
				if (octant->child[c] == 0)
					continue;
				bool deleted1 = false;
				boxWiseDelete(octant->child[c], box_range, deleted1, clear_data);
				if(deleted1) octant->child[c]=0;
			}
		}

		bool overlaps(const float * query,float sqRadius, const Octant *o)
		{
			/** \brief test if search ball S(q,r) overlaps with octant
			 * @param query   query point
			 * @param radius  "squared" radius
			 * @param o       pointer to octant
			 * @return true, if search ball overlaps with octant, false otherwise.
			 */
			// we exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant.
			float x = std::abs(query[0] - o->x) - o->extent;
			float y = std::abs(query[1] - o->y) - o->extent;
			float z = std::abs(query[2] - o->z) - o->extent;
			// float maxdist = radius + o->extent;
			// Completely outside, since q' is outside the relevant area.
			// std::abs(query[0] - o->x) - o->extent > radius
			if ((x > 0 && x*x>sqRadius) || (y > 0 && y*y>sqRadius) || (z > 0 && z*z>sqRadius))
				return false;
			int32_t num_less_extent = (x < 0) + (y < 0) + (z < 0);
			// Checking different cases:
			// a. inside the surface region of the octant.
			if (num_less_extent > 1)
				return true;
			// b. checking the corner region && edge region.
			x = std::max(x, 0.0f);
			y = std::max(y, 0.0f);
			z = std::max(z, 0.0f);

			return (x*x+y*y+z*z < sqRadius);
		}

		float sqrDist_point2octant(const float * query, const Octant *o)
		{
			// 点到box的距离，在边界上或者内部距离为0
			float x, y, z;
			x = std::max(std::abs(query[0] - o->x) - o->extent, 0.0f);
			y = std::max(std::abs(query[1] - o->y) - o->extent, 0.0f);
			z = std::max(std::abs(query[2] - o->z) - o->extent, 0.0f);
			return x*x+y*y+z*z;
		}

		bool contains(const float * query, float sqRadius, const Octant *o)
		{
			/** \brief test if search ball S(q,r) contains octant
			 * @param query    query point
			 * @param sqRadius "squared" radius
			 * @param octant   pointer to octant
			 * @return true, if search ball overlaps with octant, false otherwise.
			 */
			// we exploit the symmetry to reduce the test to test
			// whether the farthest corner is inside the search ball.
			float x = std::abs(query[0] - o->x) + o->extent;
			float y = std::abs(query[1] - o->y) + o->extent;
			float z = std::abs(query[2] - o->z) + o->extent;

			return (x*x+y*y+z*z < sqRadius);
		}

		bool inside(const float * query, float radius2, const Octant *octant)
		{
			/** \brief test if search ball S(q,r) is completely inside octant.
			 * @param query   query point
			 * @param radius2  radius r*r
			 * @param octant  point to octant.
			 * @return true, if search ball is completely inside the octant, false otherwise.
			 */
			// we exploit the symmetry to reduce the test to test
			// whether the farthest corner is inside the search ball.
			float x = octant->extent - std::abs(query[0] - octant->x);
			float y = octant->extent - std::abs(query[1] - octant->y);
			float z = octant->extent - std::abs(query[2] - octant->z);
			// octant->extent < radius + std::abs(query[0] - octant->x)
			if (x< 0 || x*x < radius2)
				return false;
			if (y< 0 || y*y < radius2)
				return false;
			if (z< 0 || z*z < radius2)
				return false;
			return true;
		}
	};


} // namespace thuni

#endif /* THUNI_OCTREE_HPP_ */
