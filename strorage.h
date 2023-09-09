#pragma once
#include <iostream>
#include <stdio.h>
#include<algorithm>
#include<cmath>
#include<vector>
#include<map>

using namespace std;

constexpr auto MAX = 1000;




 
typedef struct  Etree {
	struct Etree* child[8] = {NULL,NULL,NULL,NULL,NULL,NULL ,NULL ,NULL };
	struct Endtree* child_end[8] = { NULL,NULL,NULL,NULL,NULL,NULL ,NULL ,NULL };
	float coordinate[6];
	int depth;
}Etree;


typedef struct  Endtree {
	int point[MAX];
	int close[124];//
	float coordinate[6];
	int data=0;
}Endtree;

typedef struct point_region {//聚类时所用的存储结构
    float  xyz_inter[3];
	int usedtimes[15];
    int point[17];
    int ifselect[15];//用于标记，该点的距离是否适合加入计算
    float vector[3];//法向量
    float vector_coordinate[15][3];//投影坐标
   float point_dis[15];//先参与三维点的筛选，更新后参与二维的筛选
}point_region;


typedef  struct  endresult {
	int a[15];
	int b[15];
	int c[15];
}endresult;

int  maxfind(float a[15]);
float culculate_region(point_region* region, int n);
//void QuickSort(float arr[], int start, int end, int brr[]);
void sort_1(float* a, int n, int* b);
float  abc_cos(int a, int b, point_region* region, int n);
int sentence(float coslist[], int count, point_region* region, int pointlist[], int n);

/*邻域获取5*5*5空间*/
void   nearspace_get_2(int n, int edge[124]) {
    int temp_foredge = 0;
    int z = n / 64, y = (n - z * 64) / 8, x = (n - z * 64 - y * 8);
    int i = z - 2;
    for (i; i <= (z + 2); i++)
    {
        for (int j = (y - 2); j <= (y + 2); j++)
        {
            for (int k = (x - 2); k <= (x + 2); k++)
            {
                if (i >= 0 && j >= 0 && k >= 0)
                {
                    if (i < 8 && j < 8 && k < 8)
                    {
                        if (i != z || j != y || k != x)
                        {
                            edge[temp_foredge] = i * 64 + j * 8 + k;
                            temp_foredge++;
                        }
                        else {}
                    }
                    else { edge[temp_foredge] = 512; temp_foredge++; }
                }
                else { edge[temp_foredge] = 512; temp_foredge++; }
            }
        }
    }
}



/*邻域获取3*3*3空间*/
void   nearspace_get(int n, int *edge) {
    int temp_foredge = 0;
    int z = n / 64, y = (n - z * 64) / 8, x = (n - z * 64 - y * 8);
    int i = z - 1;
    for (i; i < (z + 2); i++)
    {
        for (int j = (y - 1); j < (y + 2); j++)
        {
            for (int k = (x - 1); k < (x + 2); k++)
            {
                if (i >= 0 && j >= 0 && k >= 0)
                {
                    if (i < 8 && j < 8 && k < 8)
                    {
                        if (i != z || j != y || k != x)
                        {
                            edge[temp_foredge] = i * 64 + j * 8 + k;
                            temp_foredge++;
                        }
                        else {}
                    }
                    else { edge[temp_foredge] = 512; temp_foredge++; }
                }
                else { edge[temp_foredge] = 512; temp_foredge++; }
            }

        }
    }
}



///*使用库函数的sort将不再使用这个sort*/
//void sort(float a[3], int b[3]) {
//    float t; int tt;
//    if (a[0] > a[1])    /*如果a大于b,借助中间变量t实现a与b值的互换*/
//    {
//        t = a[0];
//        tt = b[0];
//        a[0] = a[1];
//        b[0] = b[1];
//        a[1] = t;
//        b[1] = tt;
//
//    }
//    if (a[0] > a[2])    /*如果a大于c,借助中间变景t实现a与c值的互换*/
//    {
//        t = a[0];
//        tt = b[0];
//        a[0] = a[2];
//        b[0] = b[2];
//        a[2] = t;
//        b[2] = tt;
//    }
//    if (a[1] > a[2])    /*如果b大于c,借助中间变量t实现b与c值的互换*/
//    {
//        t = a[1];
//        tt = b[1];
//        a[1] = a[2];
//        b[1] = b[2];
//        a[2] = t;
//        b[2] = tt;
//    }
//}


void  sift(float a[15], float b, int c[17], int d) {//c[15]用于标注当前最大值
    int max = c[15];
    
    if (b > a[max]) {}
    else
    {
        a[max] = b;
        c[max] = d;
        c[15] = maxfind(a);
    }
}
/*配合maxfind一起，实现更新最近点数*/

int  maxfind(float a[15]) {//更新c[15]
    int b=0;
    float  c = a[0];
    for (int i = 1; i < 15; i++)
    {
        if (c > a[i]) {}
        else
        {
            c = a[i];
            b = i;
        }
    }
    return b;
}

//
//float ShengJin(float a, float b, float c, float d, vector<float>& X123)
//{
//    /************************************************************************/
//    /* 盛金公式求解三次方程的解
//       德尔塔f=B^2-4AC
//           这里只要了实根，虚根需要自己再整理下拿出来
//    */
//    /************************************************************************/
//    float A = b * b - 3 * a * c;
//    float B = b * c - 9 * a * d;
//    float C = c * c - 3 * b * d;
//    float f = B * B - 4 * A * C;
//    float i_value;
//    float Y1, Y2;
//    if (fabs(A) < 1e-6 && fabs(B) < 1e-6)//公式1
//    {
//        X123.push_back(-b / (3 * a));
//        X123.push_back(-b / (3 * a));
//        X123.push_back(-b / (3 * a));
//    }
//    else if (fabs(f) < 1e-6)   //公式3
//    {
//        float K = B / A;
//        X123.push_back(-b / a + K);
//        X123.push_back(-K / 2);
//        X123.push_back(-K / 2);
//    }
//    else if (f > 1e-6)      //公式2
//    {
//        Y1 = A * b + 3 * a * (-B + sqrt(f)) / 2;
//        Y2 = A * b + 3 * a * (-B - sqrt(f)) / 2;
//        float Y1_value = (Y1 / fabs(Y1)) * pow((float)fabs(Y1), 1.0 / 3);
//        float Y2_value = (Y2 / fabs(Y2)) * pow((float)fabs(Y2), 1.0 / 3);
//        X123.push_back((-b - Y1_value - Y2_value) / (3 * a));//虚根我不要
//        //虚根还是看看吧，如果虚根的i小于0.1，则判定为方程的一根吧。。。
//        i_value = sqrt(3.0) / 2 * (Y1_value - Y2_value) / (3 * a);
//        if (fabs(i_value) < 1e-1)
//        {
//            X123.push_back((-b + 0.5 * (Y1_value + Y2_value)) / (3 * a));
//        }
//    }
//    else if (f < -1e-6)   //公式4
//    {
//        float T = (2 * A * b - 3 * a * B) / (2 * A * sqrt(A));
//        float S = acos(T);
//        X123.push_back((-b - 2 * sqrt(A) * cos(S / 3)) / (3 * a));
//        X123.push_back((-b + sqrt(A) * (cos(S / 3) + sqrt(3.0) * sin(S / 3))) / (3 * a));
//        X123.push_back((-b + sqrt(A) * (cos(S / 3) - sqrt(3.0) * sin(S / 3))) / (3 * a));
//    }
//    i_value = X123[0];
//    for (size_t i = 1; i < 3; i++)
//    {
//        if (i_value > X123[i])
//        {
//            i_value = X123[i];
//        }
//
//    }
//    return i_value;
//}
//
///*三元一次方程*/
//void Three_one(float p[3],float x[3][3])
//{
//    float num12z = x[1][2] / x[0][2];
//    float num13z = x[2][2] / x[0][2];
//    float num12y = x[0][1] * num12z - x[1][1];
//    float num13y = x[0][1] * num13z - x[2][1];
//    float num12x = x[0][0] * num12z - x[1][0];
//    float num13x = x[0][0] * num13z - x[2][0];
//    float sum12 =0;
//    float sum13 = 0;
//    float num3 = num13x / num12x;
//    float val = num3 * num12y - num13y;
//    float val2 = 0;
//
//    *(p + 1) = val2 / val;							//y的值
//    *p = (sum12 - num12y * (*(p + 1))) / num12x;					//x的值
//    *(p + 2) = (0 - x[2][0] * (*(p)) - x[2][1] * (*(p + 1))) / x[2][2];	//z的值
//}/*a用来存放第一个方程(ax+by+cz=d)的系数a,b,c,d，
//    b、c依次第二、第三个，
//    p用来存放x,y,z的解*/
//




  /**
* @brief 求实对称矩阵的特征值及特征向量的雅克比法
* 利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量
* @param pMatrix				长度为n*n的数组，存放实对称矩阵
* @param nDim					矩阵的阶数
* @param pdblVects				长度为n*n的数组，返回特征向量(按列存储)
* @param dbEps					精度要求
* @param nJt					整型变量，控制最大迭代次数
* @param pdbEigenValues			特征值数组
* @return
*/
void JacbiCor(float pMatrix[9], int nDim, float* pdblVects, float* pdbEigenValues, float dbEps, int nJt)
{
	for (int i = 0; i < nDim; i++)
	{
		pdblVects[i * nDim + i] = 1.0f;
		for (int j = 0; j < nDim; j++)
		{
			if (i != j)
				pdblVects[i * nDim + j] = 0.0f;
		}
	}

	int nCount = 0;		//迭代次数
	while (1)
	{
		//在pMatrix的非对角线上找到最大元素
		float dbMax = fabs( pMatrix[1]);
		int nRow = 0;
		int nCol = 1;
		for (int i = 0; i < nDim; i++)			//行
		{
			for (int j = 0; j < nDim; j++)		//列
			{
				float d = fabs(pMatrix[i * nDim + j]);

				if ((i != j) && (d > dbMax))
				{
					dbMax = d;
					nRow = i;
					nCol = j;
				}
			}
		}

		if (dbMax < dbEps)     //精度符合要求 
			break;

		if (nCount > nJt)       //迭代次数超过限制
			break;

		nCount++;

		float dbApp = pMatrix[nRow * nDim + nRow];
		float dbApq = pMatrix[nRow * nDim + nCol];
		float dbAqq = pMatrix[nCol * nDim + nCol];

		//计算旋转角度
		float dbAngle = 0.5 * atan2(-2 * dbApq, dbAqq - dbApp);
		float dbSinTheta = sin(dbAngle);
		float dbCosTheta = cos(dbAngle);
		float dbSin2Theta = sin(2 * dbAngle);
		float dbCos2Theta = cos(2 * dbAngle);

		pMatrix[nRow * nDim + nRow] = dbApp * dbCosTheta * dbCosTheta +
			dbAqq * dbSinTheta * dbSinTheta + 2 * dbApq * dbCosTheta * dbSinTheta;
		pMatrix[nCol * nDim + nCol] = dbApp * dbSinTheta * dbSinTheta +
			dbAqq * dbCosTheta * dbCosTheta - 2 * dbApq * dbCosTheta * dbSinTheta;
		pMatrix[nRow * nDim + nCol] = 0.5 * (dbAqq - dbApp) * dbSin2Theta + dbApq * dbCos2Theta;
		pMatrix[nCol * nDim + nRow] = pMatrix[nRow * nDim + nCol];

		for (int i = 0; i < nDim; i++)
		{
			if ((i != nCol) && (i != nRow))
			{
				int u = i * nDim + nRow;	//p  
				int w = i * nDim + nCol;	//q
				dbMax = pMatrix[u];
				pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax * dbCosTheta;
				pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax * dbSinTheta;
			}
		}

		for (int j = 0; j < nDim; j++)
		{
			if ((j != nCol) && (j != nRow))
			{
				int u = nRow * nDim + j;	//p
				int w = nCol * nDim + j;	//q
				dbMax = pMatrix[u];
				pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax * dbCosTheta;
				pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax * dbSinTheta;
			}
		}

		//计算特征向量
		for (int i = 0; i < nDim; i++)
		{
			int u = i * nDim + nRow;		//p   
			int w = i * nDim + nCol;		//q
			dbMax = pdblVects[u];
			pdblVects[u] = pdblVects[w] * dbSinTheta + dbMax * dbCosTheta;
			pdblVects[w] = pdblVects[w] * dbCosTheta - dbMax * dbSinTheta;
		}

	}

	//对特征值进行排序以及重新排列特征向量,特征值即pMatrix主对角线上的元素
	std::map<float, int> mapEigen;
	for (int i = 0; i < nDim; i++)
	{
		pdbEigenValues[i] = pMatrix[i * nDim + i];

		mapEigen.insert(make_pair(pdbEigenValues[i], i));
	}

	float* pdbTmpVec = new float[nDim * nDim];
	std::map<float, int>::reverse_iterator iter = mapEigen.rbegin();
	for (int j = 0; iter != mapEigen.rend()&& j < nDim; ++iter, ++j)
	{
		for (int i = 0; i < nDim; i++)
		{
			pdbTmpVec[i * nDim + j] = pdblVects[i * nDim + iter->second];
		}

		//特征值重新排列
		pdbEigenValues[j] = iter->first;
	}

	//设定正负号
	for (int i = 0; i < nDim; i++)
	{
		float dSumVec = 0;
		for (int j = 0; j < nDim; j++)
		{
			dSumVec += pdbTmpVec[j * nDim + i];
		}
		if (dSumVec < 0)
		{
			for (int j = 0; j < nDim; j++)
				pdbTmpVec[j * nDim + i] *= -1;
		}
	}

	memcpy(pdblVects, pdbTmpVec, sizeof(float) * nDim * nDim);
	delete[]pdbTmpVec;

}



/*计算投影后的四周点到中心点距离*/
float culculate_region(point_region* region, int n) {
    float temp=0;

	for (int t = 0; t < region[n].point[16]; t++)
    {
		temp =
			region[n].point_dis[t]+temp;
    }
    
    temp = temp / region[n].point[16];//为所有距离的平均值
    return temp;
}

float  abc_cos(int a,int b,point_region* region,int n) {
    float  cos,a_b;   
    a_b=(region[n].vector_coordinate[a][0] - region[n].vector_coordinate[b][0])*(region[n].vector_coordinate[a][0] - region[n].vector_coordinate[b][0]) +
        (region[n].vector_coordinate[a][1] - region[n].vector_coordinate[b][1]) * (region[n].vector_coordinate[a][1] - region[n].vector_coordinate[b][1]) +
        (region[n].vector_coordinate[a][2] - region[n].vector_coordinate[b][2]) * (region[n].vector_coordinate[a][2] - region[n].vector_coordinate[b][2]);
    cos = (region[n].point_dis[a] * region[n].point_dis[a] + region[n].point_dis[b] * region[n].point_dis[b] - a_b) / (2 *
        region[n].point_dis[a] * region[n].point_dis[b]);
    return cos;
}



/*记得处理过的点将不再参与其他点的相关处理*/
//该函数做简单的筛选处理
void filter(point_region *region,int n,endresult *result) {
	float temp, costheta, coslist[14] = {-0.6}; //coslist[14]一维存cos值
	int count = 0, pointlist[14] = {-1};
	for (size_t i = 0; i < 15; i++)
	{
		region[n].usedtimes[i] = 2 ;
	}
	
   temp= culculate_region(region, n);
    for (size_t i = 0; i < region[n].point[16]; i++)
    {
        if (region[n].point_dis[i]<temp)
        {
            region[n].ifselect[i] = 1;//ifselect标记为1，才允许参与计算
        }
    }//小于半径1.2*temp才参与运算
    
	for (size_t z = 0; z < 15; z++)
	{
		result[n].a[z] = -1;
		result[n].c[z] = -1;
		result[n].b[z] = -1;

	}
    for (size_t i = 0; i < region[n].point[16]; i++)//按顺序一个点一个点处理
	{
		result[n].a[i] = region[n].point[i];
		
        if (region[n].ifselect[i] == 1)//region[n].point[i]>n已经确定的中心点将不再参与计算
        {
            for (size_t j =0; j < region[n].point[16]; j++) { 
                if (i!=j&& region[n].ifselect[j] == 1 )
                {
                    costheta = abc_cos(i,j,region,n);
					if (costheta > 1)
					{
						cout<<costheta << endl;
						/*for (size_t k = 0; k < region[n].point[16]; k++)
						{
							cout << region[n].vector_coordinate[k][0] << "," ;

						}
						for (size_t k = 0; k < region[n].point[16]; k++)
						{
							cout << region[n].vector_coordinate[k][1] << "," ;

						}
						for (size_t k = 0; k < region[n].point[16]; k++)
						{
							cout << region[n].vector_coordinate[k][2] << ",";

						}
						cout << n << endl;
						cout << region[n].xyz_inter[0] << region[n].xyz_inter[1] << region[n].xyz_inter[2] << endl;*/
					}
					
                    if (costheta > (-0.5)&&costheta<0.96592583)//限制角度
                   {
                        pointlist[count] = region[n].point[j];
                        coslist[count] = costheta;
                        count++;
                   }
                    //计算角度是否符合，并选用最小两个角度，还要保证这两个角度不重叠cos15=0.96592583
                    //cos120=-0.5,角度限制在15-120，搞定
                    // 
                    //问题1：如果有且选择选择次最小角度时，判别有没有包括最小角度
                    //问题2：每个i点，最多连接2个三角形(1).cos择大，(2).cos最大的索引坐标a
                    //问题3：中心点不再使用，解决方案：用>n来实现     搞定

                }
            }//对第i个点完成计算
            if (count==0)
            {
                cout << "the point have not near one" << endl;
            }
			sort_1(coslist, count, pointlist);//根据coslist的值，将数值最大的排在前面
			int a=-1;
			if (count != 0) {
				for (size_t k = 0; k < region[n].point[16]; k++)
				{
					if (pointlist[0] == region[n].point[k])
					{
						a = k;
						break;
					}
				}
				if (region[n].usedtimes[a] == 0 || region[n].usedtimes[i] == 0 || pointlist[0] < n)
				{
					result[n].b[i] = -1;
				}
				else
				{
					result[n].b[i] = pointlist[0];
					region[n].usedtimes[i] -= 1;
					region[n].usedtimes[a] -= 1;
				}
			}
			else
			{
				result[n].b[i] = -1;
			}
	//如果可以操作这两个点对应的usedtimes都要减一
			//如果pointlist[0]对应的点使用次数用完，b[]=-1,usedtime不减,pointlist[0]<n，b[]=-1
			int b= sentence(coslist, count, region, pointlist, n);
			if (b != -1)
			{
				for (size_t k = 0; k < region[n].point[16]; k++)
				{
					if (b == region[n].point[k])
					{
						a = k;
						break;
					}
				}
				if (region[n].usedtimes[a] == 0 || region[n].usedtimes[i] == 0 || b < n)
				{
					result[n].c[i] = -1;
				}
				else
				{
					result[n].c[i] = b;
					region[n].usedtimes[i] -= 1;
					region[n].usedtimes[a] -= 1;
				}
			}
			else
			{
				result[n].c[i] = b;
			}
			count = 0;
			for (size_t z = 0; z < 14; z++)
			{
				pointlist[z] = -1;
				coslist[z] = -0.6;
			}
        }//到此，第n点的n-i边数组初步处理完成
		
		
		
    }

}

void sort_1(float* a, int n, int* b) {

	int i, j;
	float temp, temp_p;
	for (i = 0; i < n; i++)
	{
		for (j = i + 1; j < n; j++)
		{
			if (a[i] < a[j])
			{
				temp = a[i];
				temp_p = b[i];
				a[i] = a[j];
				b[i] = b[j];
				a[j] = temp;
				b[j] = temp_p;
			}
		}
	}
}


int  sentence(float coslist[],int count,point_region* region,int pointlist[],int n){//防止两角在n-i同一边,不反回-1时表示在另一边找到可用点
	float costheta;
	int a=20, b=20;
	for (size_t i = 0; i < region[n].point[16]; i++)
	{
		if (region[n].point[i]==pointlist[0])
		{
			a = i;
			break;
		}
		
	}
	for (size_t i = 1; i < count; i++)
	{
		for (size_t j = 0; j < region[n].point[16]; j++)
		{
			if (region[n].point[j] == pointlist[i])
			{
				b = j;
				break;
			}

		}
		costheta = abc_cos(a, b,region, n);
		if (costheta<coslist[i])
		{
			return pointlist[i];
		}
	}
	return  -1;
}

//void QuickSort(float arr[], int start, int end, int brr[]) {//递归的时候，start和end都相对于整个arr
//    if (start >= end) { return; }
//    int temp = start;//保留start初始值
//    float pivot = arr[start];//以数组或分割后的抽象数组的第一个元素作为基准(pivot)
//    int pivot_brr = brr[start];
//    for (int i = start + 1; i <= end; i++) {//遍历arr[start]右边的元素
//        if (arr[i] < pivot) {//当arr[i]比基准小
//            arr[start] = arr[i];//小的元素放到基准所在的位置
//            brr[start] = brr[i];
//            for (int j = i; j > start; j--) {//遍历arr[start]到arr[i]，全体右移（arr[start]已经放了新的元素）
//                arr[j] = arr[j - 1];//右移
//                brr[j] = brr[j - 1];
//            }
//            arr[++start] = pivot;//由于start和i项交换，start+1项的值需要修正,完成交换后基准右移，所以start也要++
//            brr[start] = pivot_brr;
//        }
//    }//完成了一次左右分组（排序）
//
//    QuickSort(arr, temp, start - 1, brr);//对0号到start-1号元素排序，共有start个元素
//    QuickSort(arr, start + 1, end, brr);//对右边的元素排序
//} 

void statistics(endresult *result,int n) {
	int num = 0;
	ofstream outfile;
	outfile.open("demo.ply", ios::app);
	for (size_t i = 0; i < n; i++)
	{
		for (size_t j = 0; j < 15; j++)
		{
			if (result[i].a[j] != -1)
			{
				
				if (result[i].b[j]!=-1)
				{
					outfile << "\n";
					outfile << "3";
					outfile << " ";
					outfile << i;
					outfile << " ";
					outfile << result[i].a[j];
					outfile << " ";
					outfile << result[i].b[j];
					num++;

				}
				if (result[i].c[j]!=-1)
				{
					outfile << "\n";
					outfile << "3";
					outfile << " ";
					outfile << i;
					outfile << " ";
					outfile << result[i].a[j];
					outfile << " ";
					outfile << result[i].c[j];
					num++;
				}
			}

		}
		
	}
	outfile << " ";
	outfile << num;
	outfile.close();

}