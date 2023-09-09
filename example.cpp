#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>

int main(int argc, char** argv)
{

    /*��������ģ��*/
    // ����ģ�Ͷ���,�˴�����ΪPCD��ʽ�����ļ�.��������ΪPointXYZ.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //����ȡʧ�ܽ���ʾ
        return -1;
    }
    std::cerr << "���ƶ���   ���" << std::endl;


    /*�������ģ��*/
    // Normal estimation�����������ƣ�
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//����������ƶ���
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//������������ָ��
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kdtree���ڷ������ʱ��������
    tree->setInputCloud(cloud);//Ϊkdtree�������
    n.setInputCloud(cloud);//Ϊ������ƶ����������
    n.setSearchMethod(tree);//���÷������ʱ���õ�������ʽΪkdtree
    n.setKSearch(20);//���÷������ʱ,k���������ĵ���
    n.compute(*normals);  //���з������

    std::cerr << "���߼���   ���" << std::endl;

    /*���������뷨������ƴ��*/
    // ����ͬʱ������ͷ�������ݽṹ��ָ��
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    //���ѻ�õĵ����ݺͷ�������ƴ��
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);


    // ������һ��kdtree�����ؽ�
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    //Ϊkdtree�����������,�õ�����������Ϊ��ͷ���
    tree2->setInputCloud(cloud_with_normals);

    /*�����ؽ�ģ��*/
    // ����̰��������ͶӰ�ؽ�����
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //����������������,�����洢�ؽ����
    pcl::PolygonMesh triangles;

    //���ò���
    gp3.setSearchRadius(25);  // �������ӵ�֮��������루���߳�������ȷ��k���ڵ���뾶��Ĭ��Ϊ0��
    gp3.setMu(2.5);  // ��������ھ���ĳ��ӣ��ѵõ�ÿ��������������뾶��Ĭ��Ϊ0��
    gp3.setMaximumNearestNeighbors(100);  //��������������ڵ���������
    gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees ���ƽ���
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees ÿ�����ǵ����Ƕ�
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);  //��������һ�£���Ϊtrue
    // ���õ������ݺ�������ʽ
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    //��ʼ�ؽ�
    gp3.reconstruct(triangles);
    std::cerr << "�ؽ�   ���" << std::endl;

    //���ؽ�������浽Ӳ���ļ���,�ؽ������VTK��ʽ�洢
    pcl::io::saveVTKFile("mymesh.vtk", triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    fstream fs;
    fs.open("partsID.txt", ios::out);
    if (!fs)
    {
        return -2;
    }
    fs << "��������Ϊ��" << parts.size() << "\n";
    for (int i = 0; i < parts.size(); i++)
    {
        if (parts[i] != 0)
        {
            fs << parts[i] << "\n";   //���fs����
        }
    }
    

    //ͼ����ʾģ��
    //������ʾ����ָ��
    std::cerr << "��ʼ��ʾ ........" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0.6);  //���ô�����ɫ
    viewer->addPolygonMesh(triangles, "my");  //������Ҫ��ʾ���������
    //��������ģ����ʾģʽ
    //viewer->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ  
    //viewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ  
    viewer->setRepresentationToWireframeForAllActors();  //����ģ�����߿�ͼģʽ��ʾ
    viewer->addCoordinateSystem(0.1);  //��������ϵ,����Ϊ������ʾ�ߴ�
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    // Finish
    return 0;
}

