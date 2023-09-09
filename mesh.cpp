#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include "strorage.h"
#include <string.h>
#include<fstream>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>
#include<vector>
#include<math.h>
using namespace std;


void loadendtree(const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int n, float M[]);
void createtree(Etree* tree, int a_num, Endtree* endtree[]);
void  distribute_rule(float a[6], float b[8][6]);
void point_load(Endtree* endtree[], float M[], const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int n);
int  half(float M[6], float x[4], int m, int n, float duan[3], int data_num);
float  culculate(const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int a, int b);
//void  link_method(int n, Endtree* endtree[], const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, mindis* min);
void  area_pointget(point_region* region, int n, Endtree* endtree[], const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
void  vector_culculate(point_region* region, int n, const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);


int endtree_numble = 0;


int main(int argc, char** argv)
{
    int n;
    struct Etree* root = (Etree*)malloc(sizeof(struct Etree));
    
    if (root == NULL)
    {
        cout << "error" << endl;
    }

    struct  Endtree* tree[512];
    for (int i = 0; i < 512; i++)
    {
        tree[i] = (Endtree*)malloc(sizeof(struct Endtree));
        if (root == NULL)
        {
            cout << "error" << endl;
        }
    }
    root->depth = 0;
    


    /*��������ģ��*/
    // ����ģ�Ͷ���,�˴�����ΪPCD��ʽ�����ļ�.��������ΪPointXYZ.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //����ȡʧ�ܽ���ʾ
        return -1;
    }
    cerr << "���ƶ���   ���" << endl;

 /*   pcl::PLYWriter writer;
    writer.write("demo.ply", *cloud);*/


    n = cloud->width;//�ܵ���
    cout << n << endl;

    struct point_region* region;//���޸�
    region = (point_region*)malloc(n * sizeof(point_region));
    
    struct endresult* result;
    result = (endresult*)malloc(n * sizeof(endresult));

    

    loadendtree(cloud, n, root->coordinate);//�����С��Χ�ռ�
    for (size_t i = 0; i < 6; i++)
    {
        cout << root->coordinate[i] << endl;
    }

    createtree(root, 0, tree);;//�����˲���
    for (size_t i = 0; i < 512; i++)
    {
        tree[i]->data = 0;
    }



    cout << endtree_numble << endl;
    /*for ( int m = 0;  m < 512;  m++)
    {
      nearspace_get(m, tree[m]->close);
    }*/
    
    point_load(tree, root->coordinate, cloud, n);
    /*for (size_t i = 0; i < n; i++)
    {
        min->node[i].node1 =40000;
        min->node[i].node1 =40000;

    }*/
   
   /* for (size_t i = 0; i < 512; i++)
    {
        cout << tree[i]->data << endl;
    }*/

    for (size_t i = 0; i < 512; i++)
    {
       // link_method(i, tree, cloud, min);
        area_pointget(region,i,tree,cloud);

    }
    free(root); root = NULL;
    for (int i = 0; i < 512; i++)
    {
        free(tree[i]); tree[i] = NULL;
    }
    

    for (size_t z = 0; z < n; z++)
    {
        vector_culculate(region, z, cloud);
       /* if (region[z].point[16] < 2) {
            cout <<z<< "ĳ������Χ��������"<< region[z].point[16] <<endl;
            
        }*/
      
    }
   /* for (size_t i = 0; i < region[0].point[16]; i++)
    {
        cout << region[0].vector_coordinate[i][0]<<",";

    }
    cout<<"" << endl;
    for (size_t i = 0; i < region[0].point[16]; i++)
    {
        cout << region[0].vector_coordinate[i][1]<<",";

    }
    cout << "" << endl;
    for (size_t i = 0; i < region[0].point[16]; i++)
    {
        cout << region[0].vector_coordinate[i][2] << ",";

    }*/
    //cout << cloud->points[0].x << "  " << cloud->points[0].y << "  " << cloud->points[0].z;

    for (size_t i = 0; i < n; i++)
    {
       
        filter(region, i,result);
    }
    

        free(region); region = NULL;
    
   
    /*дply�ļ�*/
    
    statistics( result, n);





    return 0;
}


/*��ȡ��ǰ�ռ���С��Χ�����壬������ص������������*/
void loadendtree(const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int n, float temp[]) {
    //����˳��洢xyz���������С����
    temp[0] = cloud->points[0].x;
    temp[1] = cloud->points[0].x;
    temp[2] = cloud->points[0].y;
    temp[3] = cloud->points[0].y;
    temp[4] = cloud->points[0].z;
    temp[5] = cloud->points[0].z;
    for (size_t i = 0; i < n; i++)
    {
        if (cloud->points[i].x >= temp[0]) { temp[0] = cloud->points[i].x; }
        if (cloud->points[i].x <= temp[1]) { temp[1] = cloud->points[i].x; }
        if (cloud->points[i].y >= temp[2]) { temp[2] = cloud->points[i].y; }
        if (cloud->points[i].y <= temp[3]) { temp[3] = cloud->points[i].y; }
        if (cloud->points[i].z >= temp[4]) { temp[4] = cloud->points[i].z; }
        if (cloud->points[i].z <= temp[5]) { temp[5] = cloud->points[i].z; }
    }//����M�д�����С��Χ����������

}




void createtree(Etree* tree, int a_num, Endtree* endtree[]) {
    a_num++;
   

    for (size_t i = 0; i < 8; i++)
    {
        struct Etree* p = (Etree*)malloc(sizeof(struct Etree));
        if (p == NULL)
        {
            cout << "error" << endl;
        }

        tree->child[i] = p;
        tree->child[i]->depth = a_num;

        if (a_num < 2)
        {
            createtree(p, a_num, endtree);
        }
        else {
           
            for (size_t j = 0; j < 8; j++)
            {
                
                tree->child[i]->child_end[j] = endtree[endtree_numble];
                endtree_numble++;
            }
        }
    }
}

/*���ݷ�����򽫵�����Ӧ�ṹ��*/
void point_load(Endtree* endtree[], float M[], const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int n) {
    int a, b, c = 0;
    float d[4],e[3];
    for (size_t i = 0; i < 3; i++)
    {
        e[i] = (M[2 * i] - M[2 * i + 1]) / 8;
    }
    for (int i = 0; i < n; i++) {
        d[0] = cloud->points[i].x;
        d[1] = cloud->points[i].y;
        d[2] = cloud->points[i].z;
        d[3] = 0;//Ϊ1��־�нӽ�������
        a = half(M, d, 511, 0, e, 2);
       
       // cout << a << endl;
        b = endtree[a]->data++;
        if (b<1000)
        {
            endtree[a]->point[b] = i;
        }
       
        if (endtree[a]->data>1000)
        {
            cout << "error" << endl;
        }
        
    }
}

/*������point_load����*/
int  half(float M[6], float x[4], int m, int n,float duan[3],int data_num) {//����M��x,duan�����ᷢ���仯
    
    if (data_num==-1)
    {
       return  n;
    }
    else {
        int a = (int)((x[data_num] - M[data_num * 2+1]) / (duan[data_num]+0.0011) );//0-7
        float c = ((x[data_num] - M[data_num * 2 + 1]) - (duan[data_num]) * a)/ duan[data_num] ;
        if (c < 0.5) {
            x[3] = 1;
        }
      
        int b = (m + 1 - n) / 8;
        n = b * a + n;
        m = n + b - 1;
        data_num--;
        half(M, x, m, n, duan, data_num);
    }

}

/*���ú���*/
void  distribute_rule(float a[6], float b[8][6]) {
    float c[3];
    /*for (size_t i = 0; i < 6; i++)
    {
        cout << a[i] << endl;
    }*/
    for (size_t i = 0; i < 3; i++)
    {
        c[i] = (a[2 * i] + a[2 * i + 1]) / 2;
        //cout << c[i] << endl;

    }//����ָ��е�
    /*�ֲ�����x�Ӵ�С��y�Ӵ�С��z�Ӵ�С��x������y���*/
    b[0][0] = a[0]; b[0][1] = c[0]; b[0][2] = a[2]; b[0][3] = c[1]; b[0][4] = a[4]; b[0][5] = c[2];//xyz�����Զ
    b[1][0] = a[0]; b[1][1] = c[0]; b[1][2] = a[2]; b[1][3] = c[1]; b[1][4] = c[2]; b[1][5] = a[5];//xy���Զ
    b[2][0] = a[0]; b[2][1] = c[0]; b[2][2] = c[1]; b[2][3] = a[3]; b[2][4] = a[4]; b[2][5] = c[2];//xz���Զ
    b[3][0] = a[0]; b[3][1] = c[0]; b[3][2] = c[1]; b[3][3] = a[3]; b[3][4] = c[2]; b[3][5] = a[5];//x���Զ
    b[4][0] = c[0]; b[4][1] = a[1]; b[4][2] = a[2]; b[4][3] = c[1]; b[4][4] = a[4]; b[4][5] = c[2];//yz�����Զ
    b[5][0] = c[0]; b[5][1] = a[1]; b[5][2] = a[2]; b[5][3] = c[1]; b[5][4] = c[2]; b[5][5] = a[5];//y���Զ
    b[6][0] = c[0]; b[6][1] = a[1]; b[6][2] = c[1]; b[6][3] = a[3]; b[6][4] = a[4]; b[6][5] = c[2];//z���Զ
    b[7][0] = c[0]; b[7][1] = a[1]; b[7][2] = c[1]; b[7][3] = a[3]; b[7][4] = c[2]; b[7][5] = a[5];//������
}

//
//void  link_method(int n, Endtree* endtree[], const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, mindis* min) {
//    float distance[3];
//    int b[3], a, temp = 0, * p, num = 0, c;
//    if (endtree[n]->data <= 3 && endtree[n]->data > 0)
//    {
//        for (size_t i = 0; i < 26; i++)
//        {
//            if (endtree[n]->close[i] != 512)
//            {
//                a = endtree[n]->close[i];
//                temp = endtree[a]->data + temp;//���»�ȡ����
//            }
//        }//����ܴ�����Ԫ��
//        p = (int*)malloc(temp * sizeof(int));
//        for (size_t i = 0; i < 26; i++)
//        {
//            a = endtree[n]->close[i];
//            if (a != 512 && endtree[a]->data != 0)
//            {
//                for (int j = 0; j < endtree[a]->data; j++)
//                {
//                    p[num] = endtree[a]->point[j];
//                    num++;
//                }
//            }
//        }//�����п��ܴ�����¼��p
//        if (num != temp)
//        {
//            cout << "error" << endl;
//        }//���
//        num = 0;//
//
//
//        for (size_t i = 0; i < endtree[n]->data; i++)
//        {
//            a = endtree[n]->point[i];//��ǰ����ĵ�
//            b[0] = b[1] = b[2] = a; distance[0] = distance[1] = distance[2] = 100;//��ʼ��Ĭ��Ϊ��������
//            for (size_t j = 0; j < endtree[n]->data; j++)
//            {
//                if (i != j)//��ֹ�����Ӱ�죬�������Ϊ0
//                {
//                    b[j] = endtree[n]->point[j];
//                    distance[j] = culculate(cloud, a, endtree[n]->point[j]);
//                }
//            }sort(distance, b);//��ʼ������
//            for (size_t j = 0; j < temp; j++)
//            {
//                b[2] = p[j];
//                distance[2] = culculate(cloud, a, b[2]);
//                sort(distance, b);//����õ���С����ľ��룬�����飬���һ����Ϊ��Զ�㣬��ȥ
//            } min->node[a].node1 = b[0];
//            min->node[a].node2 = b[1];
//        } temp = 0; free(p); p = NULL;//������ɣ�temp��0��p�ͷ�
//    }
//
//
//    else  if (endtree[n]->data > 3)
//
//    {
//        for (size_t i = 0; i < endtree[n]->data; i++)
//        {
//            a = endtree[n]->point[i];//��ǰ����ĵ�
//            if (min->node[a].region != 1)
//            {
//
//                distance[0] = 100;
//                distance[2] = 100;
//                distance[1] = 100;//��ʼ������
//                b[0] = b[1] = b[2] = a;//��ʼ���ӵ�Ϊ����
//                for (size_t j = 0; j < endtree[n]->data; j++) {//����ĵ��ÿ����һһ�����
//                    if (endtree[n]->point[j] == a)
//                    {
//                        distance[2] = 100;
//                        b[2] = a;
//                    }
//                    else
//                    {
//                        b[2] = endtree[n]->point[j];
//                        distance[2] = culculate(cloud, a, endtree[n]->point[j]);
//                    }
//                    sort(distance, b);//����õ���С����ľ��룬�����飬���һ����Ϊ��Զ�㣬��ȥ
//                }
//                min->node[a].node1 = b[0];
//                min->node[a].node2 = b[1];
//            }
//            else
//            {
//
//                /*��������*/
//                for (size_t i = 0; i < 26; i++)
//                {
//                    if (endtree[n]->close[i] != 512)
//                    {
//                        c = endtree[n]->close[i];
//                        temp = endtree[c]->data + temp;//���»�ȡ����
//                    }
//                }//����ܴ�����Ԫ��
//                p = (int*)malloc(temp * sizeof(int));
//                for (size_t i = 0; i < 26; i++)
//                {
//                    c = endtree[n]->close[i];
//                    if (c != 512 && endtree[c]->data != 0)
//                    {
//                        for (int j = 0; j < endtree[c]->data; j++)
//                        {
//                            p[num] = endtree[c]->point[j];
//                            num++;
//                        }
//                    }
//                }
//                if (num != temp)
//                {
//                    cout << "error" << endl;
//                }
//                num = 0;//
//                distance[0] = 100;
//                distance[1] = 100;//��ʼ������
//                distance[2] = 100;
//                b[0] = b[1] = b[2] = a;//��ʼ���ӵ�Ϊ����;//��ʼ������
//                for (size_t j = 0; j < temp; j++)
//                {
//                    b[2] = p[j];
//                    distance[2] = culculate(cloud, a, b[2]);
//                    sort(distance, b);//����õ���С����ľ��룬�����飬���һ����Ϊ��Զ�㣬��ȥ
//                }
//                temp = 0; free(p); p = NULL;//������ɣ�temp��0��p�ͷ�
//           /*�뱾����*/
//                for (size_t j = 0; j < endtree[n]->data; j++) {//����ĵ��ÿ����һһ�����
//                    if (endtree[n]->point[j] == a)
//                    {
//                        distance[2] = 100;
//                        b[2] = a;
//                    }
//                    else
//                    {
//                        b[2] = endtree[n]->point[j];
//                        distance[2] = culculate(cloud, a, endtree[n]->point[j]);
//                    }
//                    sort(distance, b);//����õ���С����ľ��룬�����飬���һ����Ϊ��Զ�㣬��ȥ
//                }
//
//                min->node[a].node1 = b[0];
//                min->node[a].node2 = b[1];
//
//            }
//        }
//    }
//    
//
//    else if (endtree[n]->data == 0) {}//�յĿռ䲻����
//}

/*link_method����ʹ��*/

/*�������*/
float  culculate(const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int a, int b) {
    float c;
    c = (cloud->points[a].x - cloud->points[b].x) * (cloud->points[a].x - cloud->points[b].x) +
        (cloud->points[a].y - cloud->points[b].y) * (cloud->points[a].y - cloud->points[b].y) +
        (cloud->points[a].z - cloud->points[b].z) * (cloud->points[a].z - cloud->points[b].z);
    c = sqrt(c);
    return c;
}
//����1���µķ������    �㶨
//����2���������ù���  �㶨
//����3����̾��룬�ζ̾�������
//����4������



void  area_pointget(point_region *region ,int n, Endtree* endtree[], const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    float distance, max_temp;
    int b[3], a, temp = 0, * p, num = 0, num_max=35,temp_p;
    /*����*/
    if (endtree[n]->data <= num_max && endtree[n]->data > 0)//���ռ�������٣���Ҫȥ�����ռ�Ѱ��
    {   
        nearspace_get(n,endtree[n]->close);//26�ռ�
        for (size_t i = 0; i < 26; i++)
        {
            if (endtree[n]->close[i] != 512)
            {
                a = endtree[n]->close[i];

                temp = endtree[a]->data + temp;//���»�ȡ����
            }
        }//����ܴ�����Ԫ��
        temp_p = 26;
        if (temp<num_max)
        {
            temp = 0;
            nearspace_get_2(n, endtree[n]->close);//124�ռ�
            for (size_t i = 0; i < 124; i++)
            {
                if (endtree[n]->close[i] != 512)
                {
                    a = endtree[n]->close[i];
                    temp = endtree[a]->data + temp;
                }
            }//����ܴ�����Ԫ��,���ݹ��ٽ��������
            temp_p = 124;
        }
        p = (int*)malloc(temp * sizeof(int));
        for (size_t i = 0; i < temp_p; i++)
        {
            a = endtree[n]->close[i];
            if (a != 512 && endtree[a]->data != 0)
            {
                for (int j = 0; j < endtree[a]->data; j++)
                {
                    p[num] = endtree[a]->point[j];
                    num++;
                }
            }
        }//�����п��ܴ�����¼��p
        if (num != temp)
        {
            cout << "error" << endl;
        }//���
        num = 0;//����������ɣ��ȴ�ɸѡ
        /*Ӧ��ע����ǣ�p�洢�ĵ㶼�����Ǵ˿ռ��*/
        /*temp_p���˲���ʹ�ã�������ǰ15����������*/
        temp_p = 0;
        
        for (size_t i = 0; i < endtree[n]->data; i++)
        {
            a = endtree[n]->point[i];//��ǰ����ĵ�

            for (size_t j = 0; j < endtree[n]->data; j++)//����˿ռ�
            {
                if (i != j)//��ֹ�����Ӱ��
                {
                    distance = culculate(cloud, a, endtree[n]->point[j]);
                    if (temp_p<15)
                    {
                        
                        region[a].point[temp_p] = endtree[n]->point[j];//��ʱtemp_p���ڼ�¼��������
                        region[a].point_dis[temp_p] = distance;
                        temp_p++;
                            
                    }
                      max_temp = region[a].point_dis[0];
                      region[a].point[15] = 0;
                      for (int x = 1; x < temp_p; x++) {
                              if (region[a].point_dis[x] > max_temp)
                              {
                                  max_temp = region[a].point_dis[x];
                                  region[a].point[15] = x;//�ҳ�������Զ����ĵ�
                              }
                      }
                      
                    if(temp_p==15)
                    {
                        sift(region[a].point_dis, distance, region[a].point, endtree[n]->point[j]);
                    }
                        /*endtree[n]->point[j]��Ч*/
                   
                }
              
            }//��ʼ������
            region[a].point[16] = temp_p;//��¼�����е���
            for (size_t j = 0; j < temp; j++)
            {
                b[2] = p[j];
                distance = culculate(cloud, a, b[2]);
                if (temp_p < 15)
                {
                    
                    region[a].point[temp_p] = p[j];
                    region[a].point_dis[temp_p] = distance;
                    temp_p++;
              
                }
                max_temp = region[a].point_dis[0];
                region[a].point[15] = 0;
                for (int x = 1; x < temp_p; x++) {
                    if (region[a].point_dis[x] > max_temp)
                    {
                        max_temp = region[a].point_dis[x];
                        region[a].point[15] = x;//�ҳ�������Զ����ĵ�
                    }
                }
                if(temp_p==15)
                {
                    sift(region[a].point_dis, distance, region[a].point, b[2]);
                } /*b[2]��Ч*/
                
            }
            region[a].point[16] = temp_p;//��¼�����е���
            temp_p = 0;

        } temp = 0; free(p); p = NULL; //������ɣ�temp��0��p�ͷ�
    }

    else  if (endtree[n]->data > num_max)//�����㹻��������Ѱ��
    {
        nearspace_get(n, endtree[n]->close);//26�ռ�
        for (size_t i = 0; i < 26; i++)
        {
            if (endtree[n]->close[i] != 512)
            {
                a = endtree[n]->close[i];
                temp = endtree[a]->data + temp;//���»�ȡ����
            }
        }//����ܴ�����Ԫ��
        temp_p = 26;

        p = (int*)malloc(temp * sizeof(int));
        for (size_t i = 0; i < temp_p; i++)
        {
            a = endtree[n]->close[i];
            if (a != 512 && endtree[a]->data != 0)
            {
                for (int j = 0; j < endtree[a]->data; j++)
                {
                    p[num] = endtree[a]->point[j];
                    num++;
                }
            }
        }//�����п��ܴ�����¼��p
        if (num != temp)
        {
            cout << "error" << endl;
        }//���
        num = 0;//����������ɣ��ȴ�ɸѡ
        /*Ӧ��ע����ǣ�p�洢�ĵ㶼�����Ǵ˿ռ��*/
         /*temp_p���˲���ʹ�ã�������ǰ15����������*/
        temp_p = 0;

        for (size_t i = 0; i < endtree[n]->data; i++)
        {
            
            a = endtree[n]->point[i];//��ǰ����ĵ�

            for (size_t j = 0; j < endtree[n]->data; j++)//����˿ռ�
            {
                if (i != j)//��ֹ�����Ӱ�죬�������Ϊ0
                {

                    distance = culculate(cloud, a, endtree[n]->point[j]);
                    if (temp_p < 15)
                    {

                        region[a].point[temp_p] = endtree[n]->point[j];
                        region[a].point_dis[temp_p] = distance;
                        temp_p++;

                    }
                    max_temp = region[a].point_dis[0];
                    region[a].point[15] = 0;
                    for (int x = 1; x < temp_p; x++) {
                        if (region[a].point_dis[x] > max_temp)
                        {
                            max_temp = region[a].point_dis[x];
                            region[a].point[15] = x;//�ҳ�������Զ����ĵ�
                        }
                    }
                    if (temp_p== 15)
                    {
                        sift(region[a].point_dis, distance, region[a].point, endtree[n]->point[j]);
                    } /*endtree[n]->point[j]��Ч*/
                    


                }
               
            } region[a].point[16] = temp_p;//��¼�����е���
                                           //�������ڿռ䴦�����
            for (size_t j = 0; j < temp; j++)
            {
                b[2] = p[j];
                distance = culculate(cloud, a, b[2]);
                if (temp_p < 15)
                {

                    region[a].point[temp_p] = p[j];
                    region[a].point_dis[temp_p] = distance;
                    temp_p++;

                }
                max_temp = region[a].point_dis[0];
                region[a].point[15] = 0;
                for (int x = 1; x < temp_p; x++) {
                    if (region[a].point_dis[x] > max_temp)
                    {
                        max_temp = region[a].point_dis[x];
                        region[a].point[15] = x;//�ҳ�������Զ����ĵ�
                    }
                }
                if (temp_p ==15)
                {
                    sift(region[a].point_dis, distance, region[a].point,b[2]);
                } /*endtree[n]->point[j]��Ч*/
              

            }  region[a].point[16] = temp_p;//��¼�����е���
            temp_p = 0;
        } temp = 0; 
        //free(p); p = NULL;//������ɣ�temp_p��0��p�ͷ�
    }


    else if (endtree[n]->data == 0) {}//�յĿռ䲻����
}



void  vector_culculate(point_region* region, int n, const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    float temp[3] = { 0 ,0,0};//�ֱ�洢x,y��z�ľ�ֵ
    float matrix[9] = {0}, pdblVects[9] = { 0 }, pdbEigenValues[3] = { 0 };//����
    float namda;//����ֵ
    int a;//��ı��
    
    float t,T;//���ͶӰ��ʱ���õĲ���
    region[n].xyz_inter[0]=cloud->points[n].x;
    region[n].xyz_inter[1] = cloud->points[n].y;
    region[n].xyz_inter[2] = cloud->points[n].z;
    for (int i = 0; i < region[n].point[16]; i++)
    {
        a = region[n].point[i];
        temp[0] = temp[0] + cloud->points[a].x;
        temp[1] = temp[1] + cloud->points[a].y;
        temp[2] = temp[2] + cloud->points[a].z;
    }
    temp[0] = temp[0] + cloud->points[n].x;
    temp[1] = temp[1] + cloud->points[n].y;
    temp[2] = temp[2] + cloud->points[n].z;
    for (size_t i = 0; i < 3; i++)
    {
        temp[i] = temp[i] / (region[n].point[16] + 1);
    }
    for (size_t i = 0; i < region[n].point[16]; i++)//�����������������㣬��������
    {
            a = region[n].point[i];
            matrix[0] = (cloud->points[a].x - temp[0]) * (cloud->points[a].x - temp[0])+matrix[0];
            matrix[1] = matrix[3] = (cloud->points[a].x - temp[0]) * (cloud->points[a].y - temp[1])+matrix[1];
            matrix[2] = matrix[6] = (cloud->points[a].x - temp[0]) * (cloud->points[a].z - temp[2])+matrix[2];
            matrix[4] = (cloud->points[a].y - temp[1]) * (cloud->points[a].y - temp[1])+matrix[4];
            matrix[5] = matrix[7] = (cloud->points[a].y - temp[1]) * (cloud->points[a].z - temp[2])+matrix[5];
            matrix[8] = (cloud->points[a].z - temp[2]) * (cloud->points[a].z - temp[2])+matrix[8];
    }
        JacbiCor(matrix, 3, pdblVects, pdbEigenValues, 0.0000001, 10000);
        
        region[n].vector[0] = pdblVects[2];
        region[n].vector[1] = pdblVects[5];
        region[n].vector[2] = pdblVects[8];
       

        T = (region[n].vector[0] * region[n].vector[0]) + (region[n].vector[1] * region[n].vector[1]) + (region[n].vector[2] * region[n].vector[2]);
        float D = region[n].vector[0] * cloud->points[n].x + region[n].vector[1] * cloud->points[n].y + region[n].vector[2] * cloud->points[n].z;
        for (int i = 0; i < region[n].point[16]; i++)//����ͶӰ�ĵ�����
        {
            a = region[n].point[i];
            t = cloud->points[a].x *region[n].vector[0]+
                 cloud->points[a].y *region[n].vector[1]+
                cloud->points[a].z* region[n].vector[2]-D;
            t = t / T;
            region[n].vector_coordinate[i][0] =- t * region[n].vector[0] + cloud->points[a].x;
            region[n].vector_coordinate[i][1] = -t * region[n].vector[1] + cloud->points[a].y;
            region[n].vector_coordinate[i][2] = -t * region[n].vector[2] + cloud->points[a].z;
        }

        for (int i = 0; i < region[n].point[16]; i++)//����ͶӰ�������������ľ���
        {
            region[n].point_dis[i] = (region[n].vector_coordinate[i][0] - region[n].xyz_inter[0]) * (region[n].vector_coordinate[i][0] - region[n].xyz_inter[0]) +
                (region[n].vector_coordinate[i][1] - region[n].xyz_inter[1]) * (region[n].vector_coordinate[i][1] - region[n].xyz_inter[1]) +
                (region[n].vector_coordinate[i][2] - region[n].xyz_inter[2]) * (region[n].vector_coordinate[i][2] - region[n].xyz_inter[2]);
            region[n].point_dis[i] = sqrt(region[n].point_dis[i]);
        }
}