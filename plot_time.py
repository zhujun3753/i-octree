import numpy as np
import matplotlib.pyplot as plt
import sys, os
from scipy.spatial.transform import Rotation
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from matplotlib import ticker
from matplotlib.patches import  ConnectionPatch
import open3d as o3d
import matplotlib
import cv2
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42



def zone_and_linked(ax,axins,zone_left,zone_right,x,y,linked='bottom', x_ratio=0.05,y_ratio=0.05):
    """缩放内嵌图形，并且进行连线
    ax:         调用plt.subplots返回的画布。例如： fig,ax = plt.subplots(1,1)
    axins:      内嵌图的画布。 例如 axins = ax.inset_axes((0.4,0.1,0.4,0.3))
    zone_left:  要放大区域的横坐标左端点
    zone_right: 要放大区域的横坐标右端点
    x:          X轴标签
    y:          列表，所有y值
    linked:     进行连线的位置，{'bottom','top','left','right'}
    x_ratio:    X轴缩放比例
    y_ratio:    Y轴缩放比例
    """
    xlim_left = x[zone_left]-(x[zone_right]-x[zone_left])*x_ratio
    xlim_right = x[zone_right]+(x[zone_right]-x[zone_left])*x_ratio
    y_data = np.hstack([yi[zone_left:zone_right] for yi in y])
    # ylim_bottom = np.min(y_data)-(np.max(y_data)-np.min(y_data))*y_ratio
    ylim_bottom = 0.3
    ylim_top = np.max(y_data)+(np.max(y_data)-np.min(y_data))*y_ratio
    axins.set_xlim(xlim_left, xlim_right)
    axins.set_ylim(ylim_bottom, ylim_top)
    ax.plot([xlim_left,xlim_right,xlim_right,xlim_left,xlim_left],
            [ylim_bottom,ylim_bottom,ylim_top,ylim_top,ylim_bottom],"black")
    if linked == 'bottom':
        xyA_1, xyB_1 = (xlim_left,ylim_top), (xlim_left,ylim_bottom)
        xyA_2, xyB_2 = (xlim_right,ylim_top), (xlim_right,ylim_bottom)
    elif  linked == 'top':
        xyA_1, xyB_1 = (xlim_left,ylim_bottom), (xlim_left,ylim_top)
        xyA_2, xyB_2 = (xlim_right,ylim_bottom), (xlim_right,ylim_top)
    elif  linked == 'left':
        xyA_1, xyB_1 = (xlim_right,ylim_top), (xlim_left,ylim_top)
        xyA_2, xyB_2 = (xlim_right,ylim_bottom), (xlim_left,ylim_bottom)
    elif  linked == 'right':
        xyA_1, xyB_1 = (xlim_left,ylim_top), (xlim_right,ylim_top)
        xyA_2, xyB_2 = (xlim_left,ylim_bottom), (xlim_right,ylim_bottom)
    con = ConnectionPatch(xyA=xyA_1,xyB=xyB_1,coordsA="data", coordsB="data",axesA=axins,axesB=ax)
    axins.add_artist(con)
    con = ConnectionPatch(xyA=xyA_2,xyB=xyB_2,coordsA="data", coordsB="data",axesA=axins,axesB=ax)
    axins.add_artist(con)

def zone_and_linked2(ax,axins,zone_left,zone_right,x,y,linked='bottom',  ylim_bottom_ = None, ylim_top_=None, x_ratio=0.05,y_ratio=0.05):
    """缩放内嵌图形，并且进行连线
    ax:         调用plt.subplots返回的画布。例如： fig,ax = plt.subplots(1,1)
    axins:      内嵌图的画布。 例如 axins = ax.inset_axes((0.4,0.1,0.4,0.3))
    zone_left:  要放大区域的横坐标左端点
    zone_right: 要放大区域的横坐标右端点
    x:          X轴标签
    y:          列表，所有y值
    linked:     进行连线的位置，{'bottom','top','left','right'}
    x_ratio:    X轴缩放比例
    y_ratio:    Y轴缩放比例
    """
    xlim_left = x[zone_left]-(x[zone_right]-x[zone_left])*x_ratio
    xlim_right = x[zone_right]+(x[zone_right]-x[zone_left])*x_ratio
    y_data = np.hstack([yi[zone_left:zone_right] for yi in y])
    # ylim_bottom = np.min(y_data)-(np.max(y_data)-np.min(y_data))*y_ratio
    ylim_bottom = 0.3 if ylim_bottom_==None else ylim_bottom_
    ylim_top = np.max(y_data)+(np.max(y_data)-np.min(y_data))*y_ratio if ylim_top_==None else ylim_top_
    axins.set_xlim(xlim_left, xlim_right)
    axins.set_ylim(ylim_bottom, ylim_top)
    ax.plot([xlim_left,xlim_right,xlim_right,xlim_left,xlim_left],
            [ylim_bottom,ylim_bottom,ylim_top,ylim_top,ylim_bottom],"black")
    if linked == 'bottom':
        xyA_1, xyB_1 = (xlim_left,ylim_top), (xlim_left,ylim_bottom)
        xyA_2, xyB_2 = (xlim_right,ylim_top), (xlim_right,ylim_bottom)
    elif  linked == 'top':
        xyA_1, xyB_1 = (xlim_left,ylim_bottom), (xlim_left,ylim_top)
        xyA_2, xyB_2 = (xlim_right,ylim_bottom), (xlim_right,ylim_top)
    elif  linked == 'left':
        xyA_1, xyB_1 = (xlim_right,ylim_top), (xlim_left,ylim_top)
        xyA_2, xyB_2 = (xlim_right,ylim_bottom), (xlim_left,ylim_bottom)
    elif  linked == 'right':
        xyA_1, xyB_1 = (xlim_left,ylim_top), (xlim_right,ylim_top)
        xyA_2, xyB_2 = (xlim_left,ylim_bottom), (xlim_right,ylim_bottom)
    con = ConnectionPatch(xyA=xyA_1,xyB=xyB_1,coordsA="data", coordsB="data",axesA=axins,axesB=ax)
    axins.add_artist(con)
    con = ConnectionPatch(xyA=xyA_2,xyB=xyB_2,coordsA="data", coordsB="data",axesA=axins,axesB=ax)
    axins.add_artist(con)

#* 局部放大测试函数
def local_expand():
    # x坐标
    x = np.arange(1,1001)
    # 生成y轴数据，并添加随机波动
    y1 = np.log(x)
    indexs = np.random.randint(0,1000,800)
    for index in indexs:
        y1[index] += np.random.rand() - 0.5
    y2 = np.log(x)
    indexs = np.random.randint(0,1000,800)
    for index in indexs:
        y2[index] += np.random.rand() - 0.5
    y3 = np.log(x)
    indexs = np.random.randint(0,1000,800)
    for index in indexs:
        y3[index] += np.random.rand() - 0.5
    # 绘制主图
    fig, ax = plt.subplots(1,1,figsize=(12,7))
    ax.plot(x,y1,color='#f0bc94',label='trick-1',alpha=0.7)
    ax.plot(x,y2,color='#7fe2b3',label='trick-2',alpha=0.7)
    ax.plot(x,y3,color='#cba0e6',label='trick-3',alpha=0.7)
    ax.legend(loc='right')
    # plt.show()
    # 绘制缩放图
    axins = ax.inset_axes((0.4, 0.1, 0.4, 0.3))

    # 在缩放图中也绘制主图所有内容，然后根据限制横纵坐标来达成局部显示的目的
    axins.plot(x,y1,color='#f0bc94',label='trick-1',alpha=0.7)
    axins.plot(x,y2,color='#7fe2b3',label='trick-2',alpha=0.7)
    axins.plot(x,y3,color='#cba0e6',label='trick-3',alpha=0.7)

    # 局部显示并且进行连线
    zone_and_linked(ax, axins, 100, 150, x , [y1,y2,y3], 'right')

    plt.show()

#* 随机数据集对比实验结果绘图
def plot_random_dataset():
    font1 = {'family': 'Times New Roman',
    'weight': 'normal',
    'size': 26,
    }
    font2 = {'family': 'Times New Roman',
    'weight': 'normal',
    'size': 30,
    }
    data_dir = './examples/output/random_dataset'
    incremental_time_files = []
    knn_time_files = []
    radius_time_files = []
    files = os.listdir(data_dir)
    for f in files:
        if 'incremental_updates_time' in f:
            incremental_time_files.append(os.path.join(data_dir,f))
        if 'knn_time' in f:
            knn_time_files.append(os.path.join(data_dir,f))
        if 'radius_time' in f:
            radius_time_files.append(os.path.join(data_dir,f))
    incremental_time = [np.loadtxt(f) for f in incremental_time_files][0]
    knn_time = [np.loadtxt(f) for f in knn_time_files][0]
    radius_time = [np.loadtxt(f) for f in radius_time_files][0]
    tree_size = knn_time[:,0]*2_000+200_000
    print("Building: ",incremental_time[0,:][1:])
    print("mean insertion: ",incremental_time[1:,:].mean(0)[1:])
    print("mean knn: ",knn_time.mean(0)[1:])
    print("mean radius: ",radius_time.mean(0)[1:])
    # import pdb;pdb.set_trace()
    fig, axes = plt.subplots(3, 1, sharex=False, sharey=False, figsize=(12, 8))
    axes[0].plot(tree_size,incremental_time[1:,1], linewidth=2.5, label='i-Octree')
    axes[0].plot(tree_size,incremental_time[1:,2], linewidth=2.5,label='ikd-Tree')
    axes[0].plot(tree_size,incremental_time[1:,3], linewidth=2.5, label='PCL-octree')
    axes[0].legend(prop=font1,edgecolor='black',loc='lower right')
    # axes[0].set_xlabel(r'Bucket Size', font1)
    axes[0].set_ylabel(r'Run Time/ms', font1)
    axes[0].set_ylim([0,9])
    axes[0].set_title('Point Insertion', font2)
    labels = axes[0].get_xticklabels()+ axes[0].get_yticklabels()
    [label.set_fontsize(30) for label in labels]
    [label.set_fontname('Times New Roman') for label in labels]
    axes[0].grid(True, which="both", ls="-")
    axes[0].set_xticks([])

    axes[1].plot(tree_size,knn_time[:,1], linewidth=2.5, label='i-Octree')
    axes[1].plot(tree_size,knn_time[:,2], linewidth=2.5,label='ikd-Tree')
    axes[1].plot(tree_size,knn_time[:,3], linewidth=2.5, label='PCL-octree')
    axes[1].legend(prop=font1,edgecolor='black',loc='lower right')
    # axes[0].set_xlabel(r'Bucket Size', font1)
    axes[1].set_ylabel(r'Run Time/ms', font1)
    axes[1].set_ylim([0,9])
    axes[1].set_title('KNN search', font2)
    labels = axes[1].get_xticklabels() + axes[1].get_yticklabels()
    [label.set_fontsize(30) for label in labels]
    [label.set_fontname('Times New Roman') for label in labels]
    axes[1].grid(True, which="both", ls="-")
    axes[1].set_xticks([])
    axins = axes[1].inset_axes((0.2, 0.6, 0.4, 0.3))

    # 在缩放图中也绘制主图所有内容，然后根据限制横纵坐标来达成局部显示的目的
    axins.plot(tree_size,knn_time[:,1],linewidth=2.5, label='i-Octree')
    axins.plot(tree_size,knn_time[:,2],linewidth=2.5,label='ikd-Tree')
    # axins.plot(tree_size,knn_time[:,3])
    labels = axins.get_yticklabels()#+ axins.get_xticklabels()
    [label.set_fontsize(30) for label in labels]
    [label.set_fontname('Times New Roman') for label in labels]

    # 局部显示并且进行连线
    zone_and_linked(axes[1], axins, 5, 10, tree_size , [knn_time[:,1],knn_time[:,2]], 'right')

    axes[2].plot(tree_size,radius_time[:,1], linewidth=2.5, label='i-Octree')
    axes[2].plot(tree_size,radius_time[:,2], linewidth=2.5,label='ikd-Tree')
    axes[2].plot(tree_size,radius_time[:,3], linewidth=2.5, label='PCL-octree')
    axes[2].legend(prop=font1,edgecolor='black',loc='lower right')
    axes[2].set_xlabel(r'Tree size', font1)
    axes[2].set_ylabel(r'Run Time/ms', font1)
    axes[2].set_ylim([0,9])
    axes[2].set_title('Radius Neighbors Search', font2)
    
    axes[2].grid(True, which="both", ls="-")
    formatter = ticker.ScalarFormatter(useMathText=True)
    formatter.set_scientific(True) 
    formatter.set_powerlimits((0,0)) 
    axes[2].xaxis.set_major_formatter(formatter)
    axes[2].xaxis.offsetText.set_fontsize(24)
    labels = axes[2].get_xticklabels()+ axes[2].get_yticklabels()
    [label.set_fontsize(30) for label in labels]
    [label.set_fontname('Times New Roman') for label in labels]
    
    # axes[2].ticklabel_format(axis="x", style="sci", scilimits=(-1,2))
    plt.subplots_adjust(wspace =0, hspace =0.3)
    plt.subplots_adjust(left=0.07, right=0.99, top=0.95, bottom=0.12)
    # fig.savefig(os.path.join('/media/zhujun/0DFD06D20DFD06D2/SLAM/octree_test/ikd-Tree-main-github/examples/output/figures','random.pdf'),dpi=600,format='pdf')
    fig.savefig(os.path.join('./examples/output/figures','random.png'),dpi=600,format='png')
    plt.show()


plot_random_dataset()


