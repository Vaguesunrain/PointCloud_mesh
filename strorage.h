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

typedef struct point_region {//����ʱ���õĴ洢�ṹ
    float  xyz_inter[3];
	int usedtimes[15];
    int point[17];
    int ifselect[15];//���ڱ�ǣ��õ�ľ����Ƿ��ʺϼ������
    float vector[3];//������
    float vector_coordinate[15][3];//ͶӰ����
   float point_dis[15];//�Ȳ�����ά���ɸѡ�����º�����ά��ɸѡ
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

/*�����ȡ5*5*5�ռ�*/
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



/*�����ȡ3*3*3�ռ�*/
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



///*ʹ�ÿ⺯����sort������ʹ�����sort*/
//void sort(float a[3], int b[3]) {
//    float t; int tt;
//    if (a[0] > a[1])    /*���a����b,�����м����tʵ��a��bֵ�Ļ���*/
//    {
//        t = a[0];
//        tt = b[0];
//        a[0] = a[1];
//        b[0] = b[1];
//        a[1] = t;
//        b[1] = tt;
//
//    }
//    if (a[0] > a[2])    /*���a����c,�����м�侰tʵ��a��cֵ�Ļ���*/
//    {
//        t = a[0];
//        tt = b[0];
//        a[0] = a[2];
//        b[0] = b[2];
//        a[2] = t;
//        b[2] = tt;
//    }
//    if (a[1] > a[2])    /*���b����c,�����м����tʵ��b��cֵ�Ļ���*/
//    {
//        t = a[1];
//        tt = b[1];
//        a[1] = a[2];
//        b[1] = b[2];
//        a[2] = t;
//        b[2] = tt;
//    }
//}


void  sift(float a[15], float b, int c[17], int d) {//c[15]���ڱ�ע��ǰ���ֵ
    int max = c[15];
    
    if (b > a[max]) {}
    else
    {
        a[max] = b;
        c[max] = d;
        c[15] = maxfind(a);
    }
}
/*���maxfindһ��ʵ�ָ����������*/

int  maxfind(float a[15]) {//����c[15]
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
//    /* ʢ��ʽ������η��̵Ľ�
//       �¶���f=B^2-4AC
//           ����ֻҪ��ʵ���������Ҫ�Լ����������ó���
//    */
//    /************************************************************************/
//    float A = b * b - 3 * a * c;
//    float B = b * c - 9 * a * d;
//    float C = c * c - 3 * b * d;
//    float f = B * B - 4 * A * C;
//    float i_value;
//    float Y1, Y2;
//    if (fabs(A) < 1e-6 && fabs(B) < 1e-6)//��ʽ1
//    {
//        X123.push_back(-b / (3 * a));
//        X123.push_back(-b / (3 * a));
//        X123.push_back(-b / (3 * a));
//    }
//    else if (fabs(f) < 1e-6)   //��ʽ3
//    {
//        float K = B / A;
//        X123.push_back(-b / a + K);
//        X123.push_back(-K / 2);
//        X123.push_back(-K / 2);
//    }
//    else if (f > 1e-6)      //��ʽ2
//    {
//        Y1 = A * b + 3 * a * (-B + sqrt(f)) / 2;
//        Y2 = A * b + 3 * a * (-B - sqrt(f)) / 2;
//        float Y1_value = (Y1 / fabs(Y1)) * pow((float)fabs(Y1), 1.0 / 3);
//        float Y2_value = (Y2 / fabs(Y2)) * pow((float)fabs(Y2), 1.0 / 3);
//        X123.push_back((-b - Y1_value - Y2_value) / (3 * a));//����Ҳ�Ҫ
//        //������ǿ����ɣ���������iС��0.1�����ж�Ϊ���̵�һ���ɡ�����
//        i_value = sqrt(3.0) / 2 * (Y1_value - Y2_value) / (3 * a);
//        if (fabs(i_value) < 1e-1)
//        {
//            X123.push_back((-b + 0.5 * (Y1_value + Y2_value)) / (3 * a));
//        }
//    }
//    else if (f < -1e-6)   //��ʽ4
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
///*��Ԫһ�η���*/
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
//    *(p + 1) = val2 / val;							//y��ֵ
//    *p = (sum12 - num12y * (*(p + 1))) / num12x;					//x��ֵ
//    *(p + 2) = (0 - x[2][0] * (*(p)) - x[2][1] * (*(p + 1))) / x[2][2];	//z��ֵ
//}/*a������ŵ�һ������(ax+by+cz=d)��ϵ��a,b,c,d��
//    b��c���εڶ�����������
//    p�������x,y,z�Ľ�*/
//




  /**
* @brief ��ʵ�Գƾ��������ֵ�������������ſ˱ȷ�
* �����Ÿ��(Jacobi)������ʵ�Գƾ����ȫ������ֵ����������
* @param pMatrix				����Ϊn*n�����飬���ʵ�Գƾ���
* @param nDim					����Ľ���
* @param pdblVects				����Ϊn*n�����飬������������(���д洢)
* @param dbEps					����Ҫ��
* @param nJt					���ͱ�������������������
* @param pdbEigenValues			����ֵ����
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

	int nCount = 0;		//��������
	while (1)
	{
		//��pMatrix�ķǶԽ������ҵ����Ԫ��
		float dbMax = fabs( pMatrix[1]);
		int nRow = 0;
		int nCol = 1;
		for (int i = 0; i < nDim; i++)			//��
		{
			for (int j = 0; j < nDim; j++)		//��
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

		if (dbMax < dbEps)     //���ȷ���Ҫ�� 
			break;

		if (nCount > nJt)       //����������������
			break;

		nCount++;

		float dbApp = pMatrix[nRow * nDim + nRow];
		float dbApq = pMatrix[nRow * nDim + nCol];
		float dbAqq = pMatrix[nCol * nDim + nCol];

		//������ת�Ƕ�
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

		//������������
		for (int i = 0; i < nDim; i++)
		{
			int u = i * nDim + nRow;		//p   
			int w = i * nDim + nCol;		//q
			dbMax = pdblVects[u];
			pdblVects[u] = pdblVects[w] * dbSinTheta + dbMax * dbCosTheta;
			pdblVects[w] = pdblVects[w] * dbCosTheta - dbMax * dbSinTheta;
		}

	}

	//������ֵ���������Լ�����������������,����ֵ��pMatrix���Խ����ϵ�Ԫ��
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

		//����ֵ��������
		pdbEigenValues[j] = iter->first;
	}

	//�趨������
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



/*����ͶӰ������ܵ㵽���ĵ����*/
float culculate_region(point_region* region, int n) {
    float temp=0;

	for (int t = 0; t < region[n].point[16]; t++)
    {
		temp =
			region[n].point_dis[t]+temp;
    }
    
    temp = temp / region[n].point[16];//Ϊ���о����ƽ��ֵ
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



/*�ǵô�����ĵ㽫���ٲ������������ش���*/
//�ú������򵥵�ɸѡ����
void filter(point_region *region,int n,endresult *result) {
	float temp, costheta, coslist[14] = {-0.6}; //coslist[14]һά��cosֵ
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
            region[n].ifselect[i] = 1;//ifselect���Ϊ1��������������
        }
    }//С�ڰ뾶1.2*temp�Ų�������
    
	for (size_t z = 0; z < 15; z++)
	{
		result[n].a[z] = -1;
		result[n].c[z] = -1;
		result[n].b[z] = -1;

	}
    for (size_t i = 0; i < region[n].point[16]; i++)//��˳��һ����һ���㴦��
	{
		result[n].a[i] = region[n].point[i];
		
        if (region[n].ifselect[i] == 1)//region[n].point[i]>n�Ѿ�ȷ�������ĵ㽫���ٲ������
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
					
                    if (costheta > (-0.5)&&costheta<0.96592583)//���ƽǶ�
                   {
                        pointlist[count] = region[n].point[j];
                        coslist[count] = costheta;
                        count++;
                   }
                    //����Ƕ��Ƿ���ϣ���ѡ����С�����Ƕȣ���Ҫ��֤�������ǶȲ��ص�cos15=0.96592583
                    //cos120=-0.5,�Ƕ�������15-120���㶨
                    // 
                    //����1���������ѡ��ѡ�����С�Ƕ�ʱ���б���û�а�����С�Ƕ�
                    //����2��ÿ��i�㣬�������2��������(1).cos���(2).cos������������a
                    //����3�����ĵ㲻��ʹ�ã������������>n��ʵ��     �㶨

                }
            }//�Ե�i������ɼ���
            if (count==0)
            {
                cout << "the point have not near one" << endl;
            }
			sort_1(coslist, count, pointlist);//����coslist��ֵ������ֵ��������ǰ��
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
	//������Բ������������Ӧ��usedtimes��Ҫ��һ
			//���pointlist[0]��Ӧ�ĵ�ʹ�ô������꣬b[]=-1,usedtime����,pointlist[0]<n��b[]=-1
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
        }//���ˣ���n���n-i����������������
		
		
		
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


int  sentence(float coslist[],int count,point_region* region,int pointlist[],int n){//��ֹ������n-iͬһ��,������-1ʱ��ʾ����һ���ҵ����õ�
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

//void QuickSort(float arr[], int start, int end, int brr[]) {//�ݹ��ʱ��start��end�����������arr
//    if (start >= end) { return; }
//    int temp = start;//����start��ʼֵ
//    float pivot = arr[start];//�������ָ��ĳ�������ĵ�һ��Ԫ����Ϊ��׼(pivot)
//    int pivot_brr = brr[start];
//    for (int i = start + 1; i <= end; i++) {//����arr[start]�ұߵ�Ԫ��
//        if (arr[i] < pivot) {//��arr[i]�Ȼ�׼С
//            arr[start] = arr[i];//С��Ԫ�طŵ���׼���ڵ�λ��
//            brr[start] = brr[i];
//            for (int j = i; j > start; j--) {//����arr[start]��arr[i]��ȫ�����ƣ�arr[start]�Ѿ������µ�Ԫ�أ�
//                arr[j] = arr[j - 1];//����
//                brr[j] = brr[j - 1];
//            }
//            arr[++start] = pivot;//����start��i�����start+1���ֵ��Ҫ����,��ɽ������׼���ƣ�����startҲҪ++
//            brr[start] = pivot_brr;
//        }
//    }//�����һ�����ҷ��飨����
//
//    QuickSort(arr, temp, start - 1, brr);//��0�ŵ�start-1��Ԫ�����򣬹���start��Ԫ��
//    QuickSort(arr, start + 1, end, brr);//���ұߵ�Ԫ������
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