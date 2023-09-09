# PointCloud_mesh
Network construction of point clouds，based on PCL

## example.cpp 
完全使用pcl的API来实现mesh构建的
效果如下：
![屏幕截图 2023-09-10 072102](https://github.com/Vaguesunrain/PointCloud_mesh/assets/93393852/c985e94e-a3a9-4745-9cdc-67e11f81cdfe)

纹理细节
![image](https://github.com/Vaguesunrain/PointCloud_mesh/assets/93393852/ca214809-d0df-4efc-9e45-bb02314655b5)


## strorage.h和mesh.cpp
尝试只用其存储类，不使用算法API自己重新实现构建（基本上的算法就是矩阵运算，拉格朗日算子之类的）
有pcd文件读取点云，输出成ply文件，在ply文件中将面连接起来
show_ply.cpp输出图像
效果如下
![image](https://github.com/Vaguesunrain/PointCloud_mesh/assets/93393852/38b1f107-dfd7-47a5-a546-edb77f8293cd)

纹理细节
![image](https://github.com/Vaguesunrain/PointCloud_mesh/assets/93393852/6757303c-f5ae-4c84-91ce-2f13b4125fd4)



