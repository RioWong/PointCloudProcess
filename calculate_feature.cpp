#include "stdafx.h"
#include "data_struct.h"
#include "calculate_feature.h"
#include "extraction_tree.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::vector;

//using namespace std;
bool CmpDis(const LAS_POINT_PROPERTY_sim& a, const LAS_POINT_PROPERTY_sim& b)
{
    if (a.dis_from_point_plane < b.dis_from_point_plane) {
        return true;
    } else {
        return false;
    }
}

bool Cmpmin_value(const PlanSegment& a, const PlanSegment& b)
{
    if (a.min_value < b.min_value) {
        return true;
    } else {
        return false;
    }
}

int CalculateFeature::compute_iteration_number(float Pr, float epi, int h_free)
{
    int IterationNumber;
    IterationNumber = log10(1 - Pr) / log10(1 - pow(1 - epi, 3));
    return IterationNumber;
}

PlanSegment CalculateFeature::calculate_plan_parameter_3points(CloudItem points[3])
{
    PlanSegment plane;
    double XAve = 0.0, YAve = 0.0, ZAve = 0.0; //�����ɷַ�������PCA���������ɸ�����ϵ�ƽ�淨������
    CvMat A;
    boost::shared_array<double> a(new double[3 * 3]);
    //double *a = new double[3 *3];                 //������㷽���Ǽ���ĳ�����������ֵ������������
    int k = 0;
    for (int i = 0; i < 3; i++) //��С����ֵ��Ӧ�������������Ƿ�������
    {
        a[k] = points[i].x;
        a[k + 3] = points[i].y;
        a[k + 2 * 3] = points[i].z;
        k ++;
        XAve += points[i].x;
        YAve += points[i].y;
        ZAve += points[i].z;
    }
    XAve /= 3;
    YAve /= 3;
    ZAve /= 3;
    CvMat *X, *XT, *XXT, *E, *I;
    X = cvCreateMat(3, 3, CV_64FC1);
    XT = cvCreateMat(3, 3, CV_64FC1);
    XXT = cvCreateMat(3, 3, CV_64FC1);
    E = cvCreateMat(3, 3, CV_64FC1);
    I = cvCreateMat(3, 1, CV_64FC1);
    A = cvMat(3, 3, CV_64FC1, a.get());
    for (int i = 0; i < 3; i++) {
        for (int n = 0; n < 3; n++) {
            if (i % 3 == 0) {
                cvmSet(X, i, n, cvmGet(&A, i, n) - XAve);
            }
            if (i % 3 == 1) {
                cvmSet(X, i, n, cvmGet(&A, i, n) - YAve);
            }
            if (i % 3 == 2) {
                cvmSet(X, i, n, cvmGet(&A, i, n) - ZAve);
            }
        }
    }
    cvTranspose(X, XT);
    cvMatMul(X, XT, XXT);
    cvEigenVV(XXT, E, I);
    double valuemin, valuemax;
    int nummin = 0, nummax = 0;
    valuemin = cvmGet(I, 0, 0);
    valuemax = cvmGet(I, 0, 0);
    for (int i = 0; i < 3; i++) {
        if (valuemin > cvmGet(I, i, 0)) {
            valuemin = cvmGet(I, i, 0);
            nummin = i;
        }
        if (valuemax < cvmGet(I, i, 0)) {
            valuemax = cvmGet(I, i, 0);
            nummax = i;
        }
    }
    double lamada1, lamada2, lamada3;
    for (int i = 0; i < 3; i++) {
        if (i == nummin) {
            lamada3 = cvmGet(I, nummin, 0);
        } else {
            if (i == nummax) {
                lamada1 = cvmGet(I, nummax, 0);
            } else {
                lamada2 = cvmGet(I, i, 0);
            }

        }
    }
    /*����ƽ�淽�̵��ĸ�����*/
    plane.normal_x = cvmGet(E, nummin, 0);
    plane.normal_y = cvmGet(E, nummin, 1);
    plane.normal_z = cvmGet(E, nummin, 2);
    plane.Distance = - (plane.normal_x * XAve + plane.normal_y * YAve + plane.normal_z * ZAve);
    cvReleaseMat(&X);
    cvReleaseMat(&XT);
    cvReleaseMat(&XXT);
    cvReleaseMat(&E);
    cvReleaseMat(&I);
    return plane;
}

PlanSegment CalculateFeature::calculate_plan_parameter_h_points(CloudPtr cloud)
{
    int h = cloud->points.size();
    PlanSegment plane;
    double XAve = 0.0, YAve = 0.0, ZAve = 0.0; //�����ɷַ�������PCA���������ɸ�����ϵ�ƽ�淨������
    CvMat A;

    // �ڴ�й©
    boost::shared_array<double> a(new double[3 * h]);
    // double *a = new double[3 *h];                 //������㷽���Ǽ���ĳ�����������ֵ������������
    int k = 0;
    //��С����ֵ��Ӧ�������������Ƿ�������
    for (int i = 0; i < h; i++) {
        a[k] = cloud->points[i].x;
        a[k + h] = cloud->points[i].y;
        a[k + 2 * h] = cloud->points[i].z;
        k ++;
        XAve += cloud->points[i].x;
        YAve += cloud->points[i].y;
        ZAve += cloud->points[i].z;
    }
    XAve /= h;
    YAve /= h;
    ZAve /= h;
    CvMat *X, *XT, *XXT, *E, *I;
    X = cvCreateMat(3, h, CV_64FC1);
    XT = cvCreateMat(h, 3, CV_64FC1);
    XXT = cvCreateMat(3, 3, CV_64FC1);
    E = cvCreateMat(3, 3, CV_64FC1);
    I = cvCreateMat(3, 1, CV_64FC1);
    A = cvMat(3, h, CV_64FC1, a.get());
    for (int i = 0; i < 3; i++) {
        for (int n = 0; n < h; n++) {
            if (i % 3 == 0) {
                cvmSet(X, i, n, cvmGet(&A, i, n) - XAve);
            }
            if (i % 3 == 1) {
                cvmSet(X, i, n, cvmGet(&A, i, n) - YAve);
            }
            if (i % 3 == 2) {
                cvmSet(X, i, n, cvmGet(&A, i, n) - ZAve);
            }
        }
    }
    cvTranspose(X, XT);
    cvMatMul(X, XT, XXT);
    cvEigenVV(XXT, E, I);
    double valuemin, valuemax;
    int nummin = 0, nummax = 0;
    valuemin = cvmGet(I, 0, 0);
    valuemax = cvmGet(I, 0, 0);
    for (int i = 0; i < 3; i++) {
        if (valuemin > cvmGet(I, i, 0)) {
            valuemin = cvmGet(I, i, 0);
            nummin = i;
        }
        if (valuemax < cvmGet(I, i, 0)) {
            valuemax = cvmGet(I, i, 0);
            nummax = i;
        }
    }
    double lamada1, lamada2, lamada3;
    for (int i = 0; i < 3; i++) {
        if (i == nummin) {
            lamada3 = cvmGet(I, nummin, 0);
        } else {
            if (i == nummax) {
                lamada1 = cvmGet(I, nummax, 0);
            } else {
                lamada2 = cvmGet(I, i, 0);
            }

        }
    }
    /*����ƽ�淽�̵��ĸ�����*/
    plane.normal_x = cvmGet(E, nummin, 0);
    plane.normal_y = cvmGet(E, nummin, 1);
    plane.normal_z = cvmGet(E, nummin, 2);
    plane.Distance = - (plane.normal_x * XAve + plane.normal_y * YAve + plane.normal_z * ZAve);
    plane.min_value = lamada3;
    plane.curvature = lamada3 / (lamada1 + lamada2 + lamada3);
    cvReleaseMat(&X);
    cvReleaseMat(&XT);
    cvReleaseMat(&XXT);
    cvReleaseMat(&E);
    cvReleaseMat(&I);
    return plane;
}

boost::shared_array<LAS_POINT_PROPERTY> CalculateFeature::calculate_plan_parameter_rpca(CloudPtr cloud, double Radius, float Pr, float epi)
{
    srand((unsigned)time(NULL));
    boost::shared_array<LAS_POINT_PROPERTY> PointProperty(new LAS_POINT_PROPERTY[cloud->size()]);
    //LAS_POINT_PROPERTY * PointProperty = new LAS_POINT_PROPERTY[cloud->size()];
    //LAS_POINT_PROPERTY_sim * PointProperty_N;
    /*����KD��*/
    // TODO: KD tree
    KdTreeFLANN<CloudItem> kdtree;
   // KdTree kdtree;
    kdtree.setInputCloud(cloud);
    /*����ÿһ����;*/
// 	time_t timeBegin, timeEnd;
// 	timeBegin = time(NULL);
#pragma omp parallel for schedule(dynamic)
    for (int j = 0; j < cloud->size(); j++) {
        CloudPtr cloud_h(new Cloud);
        CloudPtr cloud_final(new Cloud);
        vector<int> pointIdxSearch;//�洢�����ĵ��;
        vector<double> pointDistance;//�洢����㵽������ľ���;
        CloudItem searchPoint;//������;
        PointProperty[j].PointID = j;
        searchPoint = cloud->points[j];
        //�ҵ�ÿ����������
        //�洢�����ĸ���;
        int N = kdtree.nearestKSearch(searchPoint, 20, pointIdxSearch, pointDistance);
        boost::shared_array<LAS_POINT_PROPERTY_sim> PointProperty_N(new LAS_POINT_PROPERTY_sim[N]);
        if (N > 3) {
            /*ȥ��Ĺ���;*/
            int IterationNumber;
            int h_free;
            h_free = 2.0 / 3 * N;
            IterationNumber = compute_iteration_number(Pr, epi, h_free);
            //cout<<IterationNumber<<endl;
            vector<PlanSegment> planes;
            /*��������ƽ�����;*/
            for (int i = 0; i < IterationNumber; i++) {
                /*������С����ֵ;*/
                CloudItem points[3];
                int num[3];
                for (int n = 0; n < 3; n++) {
                    num[n] = rand() % N;

                    points[n] = cloud->points[pointIdxSearch[num[n]]];
                }

                if (num[0] == num[1] || num[0] == num[2] || num[1] == num[2])
                    continue;
                PlanSegment a_plane;
                a_plane = calculate_plan_parameter_3points(points);
                /*����N ���㵽ƽ��ľ���;*/
                for (int n = 0; n < N; n++) {
                    TreeExtration tree_ex;
                    PointProperty_N[n].dis_from_point_plane = tree_ex.compute_distance_from_point_to_plane(&cloud->points[pointIdxSearch[n]],
                                                                                                      a_plane.normal_x, a_plane.normal_y, a_plane.normal_z, a_plane.Distance);
                    PointProperty_N[n].x = cloud->points[pointIdxSearch[n]].x;
                    PointProperty_N[n].y = cloud->points[pointIdxSearch[n]].y;
                    PointProperty_N[n].z = cloud->points[pointIdxSearch[n]].z;
                }

                /*���ݵ㵽ƽ����������ҵ�0-��h-1������;*/
                std::sort(PointProperty_N.get(), PointProperty_N.get() + N, CmpDis);
                /*��h�������ƽ��;*/
                for (int n = 0; n < h_free; n++) {
                    CloudItem temp_point;
                    temp_point.x = PointProperty_N[n].x;
                    temp_point.y = PointProperty_N[n].y;
                    temp_point.z = PointProperty_N[n].z;
                    cloud_h->points.push_back(temp_point);
                }
                a_plane = calculate_plan_parameter_h_points(cloud_h);
                cloud_h->points.clear();
                planes.push_back(a_plane);
            }
            /*ѡ����С����ֵ��С��һ�ε�����ƽ����Ͻ������Ϊ�������ƽ�����;*/
            sort(planes.begin(), planes.end(), Cmpmin_value);
            PlanSegment final_plane;
            final_plane = planes[0];
            planes.clear();
            vector<float> distance;
            vector<float> distance_sort;
            for (int n = 0; n < N; n++) {
                /*����㵽ƽ��ľ���;*/
                TreeExtration tree_ex;
                float dis_from_point_plane;
                dis_from_point_plane = tree_ex.compute_distance_from_point_to_plane(&cloud->points[pointIdxSearch[n]],
                                                                               final_plane.normal_x, final_plane.normal_y, final_plane.normal_z, final_plane.Distance);

                distance.push_back(dis_from_point_plane);
                distance_sort.push_back(dis_from_point_plane);
            }
            sort(distance_sort.begin(), distance_sort.end());
            float distance_median;
            distance_median = distance_sort[N / 2];
            distance_sort.clear();
            float MAD;
            vector<float> temp_MAD;

            for (int n = 0; n < N; n++) {
                /*����ÿ�����temp_MAD;*/
                temp_MAD.push_back(std::abs(distance[n] - distance_median));
            }

            sort(temp_MAD.begin(), temp_MAD.end());
            MAD = 1.4826 * temp_MAD[N / 2];
            if (MAD == 0) {
                for (int n = 0; n < N; n++) {
                    CloudItem temp_point;
                    temp_point.x = cloud->points[pointIdxSearch[n]].x;
                    temp_point.y = cloud->points[pointIdxSearch[n]].y;
                    temp_point.z = cloud->points[pointIdxSearch[n]].z;
                    cloud_final->points.push_back(temp_point);
                }
            } else {
                for (int n = 0; n < N; n++) {
                    /*����ÿ�����Rz��������Rzȥ������;*/
                    float Rz;
                    Rz = (std::abs(distance[n] - distance_median)) / MAD;
                    if (Rz < 2.5) {
                        CloudItem temp_point;
                        temp_point.x = cloud->points[pointIdxSearch[n]].x;
                        temp_point.y = cloud->points[pointIdxSearch[n]].y;
                        temp_point.z = cloud->points[pointIdxSearch[n]].z;
                        cloud_final->points.push_back(temp_point);
                    }
                }
            }
            /*���÷����������ƽ���������Ϊ���ս��;*/
            if (cloud_final->points.size() > 3) //������Ⱥ��
            {
                final_plane = calculate_plan_parameter_h_points(cloud_final);

                //cout<<"noise number: "<<20-cloud_final->points.size()<<endl;
                PointProperty[j].PointID = j;
                PointProperty[j].normal_x = final_plane.normal_x;
                PointProperty[j].normal_y = final_plane.normal_y;
                PointProperty[j].normal_z = final_plane.normal_z;
                PointProperty[j].Distance = final_plane.Distance;
                PointProperty[j].curvature = final_plane.curvature;
            } else {
                PointProperty[j].PointID = j;
                PointProperty[j].normal_x = 0.0;
                PointProperty[j].normal_y = 0.0;
                PointProperty[j].normal_z = 0.0;
                PointProperty[j].Distance = 0.0;
                PointProperty[j].curvature = 1.0;
            }
            cloud_final->clear();
        } else {
            PointProperty[j].PointID = j;
            PointProperty[j].normal_x = 0.0;
            PointProperty[j].normal_y = 0.0;
            PointProperty[j].normal_z = 0.0;
            PointProperty[j].Distance = 0.0;
            PointProperty[j].curvature = 1.0;
        }	
    }
// 	timeEnd = time(NULL);
// 	printf("%d\n", timeEnd - timeBegin);
    return PointProperty;
}
CLOUD_BLEND_DOUBLE_NAMESPACE_END
