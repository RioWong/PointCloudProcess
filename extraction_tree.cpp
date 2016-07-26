#include "stdafx.h"
#include "data_struct.h"
#include "extraction_tree.h"
#include "kd_tree.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
#define  UNSEGMENTATION -1
//using namespace std;
using std::vector;
using std::set;
using std::less;
using std::deque;

struct curvatureSort //�洢�����ʵ���������������
{
    int PointPropertyID; //��Ӧ�ĵ��
    double curvature; //�õ������
};

bool CmpPointCurvature(const curvatureSort& pn1, const curvatureSort& pn2)
{
    if (pn1.curvature < pn2.curvature)
        return true;
    else
        return false;
}

bool CmpPlanePointNum(const PlanSegment& p1, const PlanSegment& p2)
{
    if (p1.PointID.size() > p2.PointID.size())
        return true;
    else
        return false;
}

float TreeExtration::compute_included_angle_between_vector(float vx1, float vy1, float vz1, float vx2, float vy2, float vz2)
{
    /*�������������ӵ㷨�����ļн�;*/
    float n_n1 = vx1 * vx2 + vy1 * vy2 + vz1 * vz2;
    float n_n = sqrt(vx1 * vx1 + vy1 * vy1 + vz1 * vz1);
    float n1_n1 = sqrt(vx2 * vx2 + vy2 * vy2 + vz2 * vz2);
    float Cosnormal = std::abs(n_n1 / (n_n * n1_n1));
    return Cosnormal;
}

/*a_point��ƽ����һ��,a,b,c,d��ƽ�淽�̵�ϵ��;*/
float TreeExtration::compute_distance_from_point_to_plane(CloudItem* a_point, float a, float b, float c, float d)
{
    /*��������㵽���ӵ�����ƽ��Ĵ�ֱ����;*/
    float x1, y1, z1;
    //ƽ����һ�㣨x1,y1,z1��;
    x1 = (*a_point).x;
    y1 = (*a_point).y;
    z1 = (*a_point).z;
    float dis;
    double g = sqrt(a * a + b * b + c * c);//��ƽ�淽��ϵ��nx,ny,nz��ƽ���͵Ŀ�ƽ��;
    double f1 = a * x1;
    double f2 = b * y1;
    double f3 = c * z1;
    double f4 = d;
    double f = std::abs(f1 + f2 + f3 + f4);
    dis = (f / g);
    return dis;
}

vector<PlanSegment> TreeExtration::region_growning(boost::shared_array<LAS_POINT_PROPERTY> PointProperty, CloudPtr& Cloud, double distanceT, double radius, double cosfaT)
{
    // 	std::cout<<"��ʼ��������"<<std::endl;
    // 	vector <PlanSegment> SegmentationResult;
    // 	/*����KD��*/
    // 	pcl::KdTreeFLANN<pcl::PointXYZRGB>kdtree;
    // 	std::vector<int>pointIdxSearch;
    // 	std::vector<float>pointDistance;
    // 	kdtree.setInputCloud(Cloud);
    // 	/*����UnSegment�Լ�UnSegment�ĵ��������洢δ�ָ�ĵ��;*/
    // 	PlanSegment TempSeg;
    // 	set<int,less<int> >UnSegment;
    // 	set<int,less<int> >::iterator iterUnseg;
    // 
    // 	vector<curvatureSort> curvaturesort;
    // 	curvaturesort.resize(Cloud->points.size());
    // 	for (int i=0;i<Cloud->points.size();i++)
    // 	{
    // 		UnSegment.insert(i);
    // 		PointProperty[i].SegmentID=UNSEGMENTATION;
    // 		curvaturesort[i].PointPropertyID=i;
    // 		curvaturesort[i].curvature=PointProperty[i].curvature;
    // 	}
    // 	/*��������deque,���ڴ洢һ���ָ���������ӵ�;*/
    // 	deque <int>seed;
    // 	int labelOfSeg =0;
    // 	/*�ѵ��������鰴����������*/
    // 	std::sort(curvaturesort.begin(),curvaturesort.end(),CmpPointCurvature);
    // 	std::cout<<curvaturesort[0].curvature<<"  "<<curvaturesort[curvaturesort.size()-1].curvature<<std::endl;
    // 	while(!UnSegment.empty())
    // 	{
    // 		/*ѡȡʣ�����ƽ����ϲв���С����С��Ϊ���ӵ�;*/
    // 		iterUnseg = UnSegment.begin();
    // 		int minNum =* iterUnseg;
    // 		/*�����ӵ�ѹ��segment��seed��;*/
    // 		PointProperty[minNum].SegmentID=labelOfSeg;
    // 		seed.push_back(minNum);
    // 		TempSeg.SegmentID=labelOfSeg;
    // 		TempSeg.PointID.push_back(PointProperty[minNum].PointID);
    // 		/*��UnSegment��ȥ����minNum��*/
    // 		UnSegment.erase(minNum);
    // 		/*�õ�������ӵ�ķ���nx,ny,nz��������px,py,pz;*/
    // 		double nx,ny,nz;
    // 		nx=PointProperty[minNum].normal_x;
    // 		ny=PointProperty[minNum].normal_y;
    // 		nz=PointProperty[minNum].normal_z;
    // 		while(!seed.empty())
    // 		{	
    // 			double nx_seed,ny_seed,nz_seed,px_seed,py_seed,pz_seed,d_seed;
    // 			int point_id=seed.front();
    // 			seed.pop_front();//���ӵ㵯��;
    // 			nx_seed=PointProperty[point_id].normal_x;
    // 			ny_seed=PointProperty[point_id].normal_y;
    // 			nz_seed=PointProperty[point_id].normal_z;
    // 			d_seed=PointProperty[point_id].Distance;
    // 			pcl::PointXYZRGB searchPoint;
    // 			/*�����ӵ�����긳ֵ��searchPoint;*/
    // 			searchPoint=Cloud->points[PointProperty[minNum].PointID];
    // 			int N=kdtree.nearestKSearch(searchPoint,20,pointIdxSearch,pointDistance);
    // 			for (int i=0;i<N;i++)
    // 			{
    // 				/*��״������;*/
    // 				if (PointProperty[pointIdxSearch[i]].SegmentID==UNSEGMENTATION)
    // 				{
    // 					float nx1=PointProperty[pointIdxSearch[i]].normal_x;
    // 					float ny1=PointProperty[pointIdxSearch[i]].normal_y;
    // 					float nz1=PointProperty[pointIdxSearch[i]].normal_z;
    // 					/*���㵱ǰ�����ͳ�ʼ���ӵ㷨�����ļн�����ֵ;*/
    // 					float Cosnormal=compute_included_angle_between_vector(nx,ny,nz,nx1,ny1,nz1);	
    // 					/*��������㵽���ӵ�����ƽ��Ĵ�ֱ����;*/
    // 					//ƽ����һ�㣨x2,y2,z2��;
    // 					pcl::PointXYZRGB point_outside_plane;
    // 					point_outside_plane=Cloud->points[pointIdxSearch[i]];
    // 					float dis=compute_distance_from_point_to_plane(&point_outside_plane,nx_seed,ny_seed,nz_seed,d_seed);
    // 					if (Cosnormal>cosfaT&&dis<distanceT)//��״������������;
    // 					{
    // 						PointProperty[pointIdxSearch[i]].SegmentID=labelOfSeg;
    // 						TempSeg.PointID.push_back(pointIdxSearch[i]);
    // 						UnSegment.erase(pointIdxSearch[i]);
    // 						seed.push_back(pointIdxSearch[i]);		
    // 					} 
    // 					else
    // 					{
    // 						/*cout<<setiosflags(ios::fixed)<<setprecision(3)<<Cosnormal<<" "
    // 							<<setiosflags(ios::fixed)<<setprecision(3)<<dis<<endl;*/
    // 					}
    // 				}
    // 			}
    // 		}
    // 		if (TempSeg.PointID.size()>5)
    // 		{
    // 			SegmentationResult.push_back(TempSeg);		
    // 			labelOfSeg++;
    // 		}
    // 		
    // 		else
    // 		{
    // 			for (int ii=0;ii<TempSeg.PointID.size();ii++)
    // 			{
    // 				PointProperty[TempSeg.PointID[ii]].SegmentID=UNSEGMENTATION;
    // 			}
    // 		}
    // 		TempSeg.PointID.clear();
    // 		seed.clear();	
    // 	}
    // 	std::cout<<SegmentationResult.size()<<std::endl;
    // 	return SegmentationResult;
    vector<PlanSegment> SegmentationResult;
    /*����KD��*/
    KdTreeFLANN<CloudItem> kdtree;
    //KdTree kdtree;
    std::vector<int> pointIdxSearch;
    std::vector<double> pointDistance;
    kdtree.setInputCloud(Cloud);
    /*����UnSegment�Լ�UnSegment�ĵ��������洢δ�ָ�ĵ��;*/
    PlanSegment TempSeg;
    set<int, less<int> > UnSegment;
    set<int, less<int> >::iterator iterUnseg;
    for (int i = 0; i < Cloud->size(); i++) {
        UnSegment.insert(i);
        PointProperty[i].SegmentID = UNSEGMENTATION;
    }
    /*��������deque,���ڴ洢һ���ָ���������ӵ�;*/
    deque<int> seed;
    int labelOfSeg = 0;
    while (!UnSegment.empty()) {
        /*ѡȡʣ�����ƽ����ϲв���С����С��Ϊ���ӵ�;*/
        iterUnseg = UnSegment.begin();
        int minNum = * iterUnseg;
        /*�����ӵ�ѹ��segment��seed��;*/
        if (PointProperty[minNum].curvature < 0.005) {
            UnSegment.erase(minNum);
            PointProperty[minNum].SegmentID = labelOfSeg;
            seed.push_back(minNum);
            TempSeg.SegmentID = labelOfSeg;
            TempSeg.PointID.push_back(minNum);
            /*��UnSegment��ȥ����minNum��*/
            UnSegment.erase(minNum);
            /*�õ�������ӵ�ķ���nx,ny,nz��������px,py,pz;*/
            double nx, ny, nz, nd;
            nx = PointProperty[minNum].normal_x;
            ny = PointProperty[minNum].normal_y;
            nz = PointProperty[minNum].normal_z;
            nd = PointProperty[minNum].Distance;
            while (!seed.empty()) {
                double nx_seed, ny_seed, nz_seed, px_seed, py_seed, pz_seed, d_seed;
                int point_id = seed.front();
                seed.pop_front();//���ӵ㵯��;
                nx_seed = PointProperty[point_id].normal_x;
                ny_seed = PointProperty[point_id].normal_y;
                nz_seed = PointProperty[point_id].normal_z;
                d_seed = PointProperty[point_id].Distance;
                CloudItem searchPoint;
                /*�����ӵ�����긳ֵ��searchPoint;*/
                searchPoint = Cloud->points[point_id];
                searchPoint = Cloud->points[PointProperty[point_id].PointID];
                int N = kdtree.nearestKSearch(searchPoint, 50, pointIdxSearch, pointDistance);
                //std::vector<CloudItem> k_points;
                //kdtree.nearestKSearch(searchPoint, 50, k_points);
                //int N = k_points.size();
                for (int i = 0; i < N; i++) {
                    /*��״������;*/
                    if (PointProperty[pointIdxSearch[i]].SegmentID == UNSEGMENTATION) {
                        float nx1 = PointProperty[pointIdxSearch[i]].normal_x;
                        float ny1 = PointProperty[pointIdxSearch[i]].normal_y;
                        float nz1 = PointProperty[pointIdxSearch[i]].normal_z;
                        /*���㵱ǰ�����ͳ�ʼ���ӵ㷨�����ļн�����ֵ;*/
                        float Cosnormal = compute_included_angle_between_vector(nx, ny, nz, nx1, ny1, nz1);
                        /*��������㵽���ӵ�����ƽ��Ĵ�ֱ����;*/
                        //ƽ����һ�㣨x2,y2,z2��;
                        CloudItem point_outside_plane;
                        point_outside_plane = Cloud->points[pointIdxSearch[i]];
                        float dis = compute_distance_from_point_to_plane(&point_outside_plane, nx_seed, ny_seed, nz_seed, d_seed);
                        if (Cosnormal > cosfaT && dis < distanceT)//��״������������;
                        {
                            PointProperty[pointIdxSearch[i]].SegmentID = labelOfSeg;
                            TempSeg.PointID.push_back(pointIdxSearch[i]);
                            UnSegment.erase(pointIdxSearch[i]);
                            seed.push_back(pointIdxSearch[i]);
                        } else {
                            /*cout<<setiosflags(ios::fixed)<<setprecision(3)<<Cosnormal<<" "
                                <<setiosflags(ios::fixed)<<setprecision(3)<<dis<<endl;*/
                        }
                    }
                }
            }
            if (TempSeg.PointID.size() > 5) {
                //count_pca_value(TempSeg,Cloud);
                TempSeg.normal_x = nx;
                TempSeg.normal_y = ny;
                TempSeg.normal_z = nz;
                TempSeg.Distance = nd;
                SegmentationResult.push_back(TempSeg);
                labelOfSeg++;
            } else {
                for (int ii = 0; ii < TempSeg.PointID.size(); ii++) {
                    PointProperty[TempSeg.PointID[ii]].SegmentID = UNSEGMENTATION;
                }
            }
            TempSeg.PointID.clear();
            seed.clear();
        } else
            UnSegment.erase(minNum);

    }
    return SegmentationResult;
}

void TreeExtration::refine_planes(std::vector<PlanSegment>& planes, const CloudPtr& cloud)
{
    std::vector<PlanSegment> dst_planes;
    std::cout << "��ʼ��Ƭ�ϲ�" << planes.size() << std::endl;
    std::sort(planes.begin(), planes.end(), CmpPlanePointNum);
    set<int, less<int> > UnSegment;
    set<int, less<int> >::iterator iterUnseg;
    vector<vector<int> > planeIDs; //��ϲ���ƽ��
    vector<int> templaneID;
    for (int i = 0; i < planes.size(); i++) {
        UnSegment.insert(i);
    }
    /*��������deque,���ڴ洢һ���ϲ���������ӵ�;*/
    deque<int> seed;
    while (!UnSegment.empty()) {
        /*ѡȡʣ�����ƽ����ϲв���С����С��Ϊ���ӵ�;*/
        iterUnseg = UnSegment.begin();
        int minNum = * iterUnseg;
        UnSegment.erase(minNum);
        seed.push_back(minNum);
        int currentplaneid = minNum;
        templaneID.push_back(minNum);
        while (!seed.empty()) {
            currentplaneid = seed.front();
            UnSegment.erase(currentplaneid);
            seed.pop_front();//���ӵ㵯��;
            // set<int,less<int> >::iterator i=UnSegment.begin();	
            // while(	)
            //����ѭ����ʽ������������
            for (set<int, less<int> >::iterator i = UnSegment.begin(); i != UnSegment.end();) {
                int distplaneid = *i;
                /*cout<<distplaneid<<endl;*/
                if (judge_two_planes_coplane(planes[currentplaneid], planes[distplaneid], cloud)) {
                    UnSegment.erase(i++);
                    //i = UnSegment.erase(i);
                    seed.push_back(*i);
                    templaneID.push_back(*i);
                } else {
                    ++i;
                }
            }

        }
        planeIDs.push_back(templaneID);
        templaneID.clear();
        seed.clear();
    }
    for (int i = 0; i < planeIDs.size(); i++) {
        for (int j = 0; j < planeIDs[i].size(); j++) {
            merge_plane(planes[planeIDs[i][0]], planes[planeIDs[i][j]], cloud);
        }
        dst_planes.push_back(planes[planeIDs[i][0]]);
    }
    planes.clear();
    planes = dst_planes;

    //	std::vector<int> unsettledplaneids;
    // 	int currentplaneid,distplaneid;    //��ǰ�棬Ŀ����
    // 	for (int i = 0; i < planes.size(); ++i)
    // 	{
    // 		unsettledplaneids.push_back(i);
    // 	}
    // 	while (!unsettledplaneids.empty())
    // 	{
    // 		currentplaneid=unsettledplaneids[0];      //��ȡ��ǰ�������id;
    // 		for (auto i=unsettledplaneids.begin()+1;i!=unsettledplaneids.end();)
    // 		{
    // 			distplaneid=*i;
    // 			if (judge_two_planes_coplane(planes[currentplaneid],planes[distplaneid],cloud))
    // 			{
    // 				i=unsettledplaneids.erase(i);
    // 				merge_plane(planes[currentplaneid],planes[distplaneid],cloud);
    // 			}
    // 			else
    // 			{
    // 				i++;
    // 			}
    // 		}
    // 		if(planes[currentplaneid].PointID.size()>20)
    // 		{
    // 			//count_pca_value(planes[currentplaneid],cloud);
    // 			dst_planes.push_back(planes[currentplaneid]);
    // 		    unsettledplaneids.erase(unsettledplaneids.begin());
    // 		}
    // 		else
    // 			unsettledplaneids.erase(unsettledplaneids.begin());
    // 
    // 	}
    // 	planes.clear();
    // 	planes=dst_planes;
}

bool TreeExtration::judge_two_planes_coplane(PlanSegment& p1, PlanSegment& p2, const CloudPtr& cloud)
{
    // First, check the normal
    count_pca_value(p1, cloud);
    count_pca_value(p2, cloud);
    float deno1 = std::sqrt(p1.normal_x * p1.normal_x + p1.normal_y * p1.normal_y + p1.normal_z * p1.normal_z);
    float deno2 = std::sqrt(p2.normal_x * p2.normal_x + p2.normal_y * p2.normal_y + p2.normal_z * p2.normal_z);
    float cos_ang = std::abs(p1.normal_x * p2.normal_x + p1.normal_y * p2.normal_y + p1.normal_z * p2.normal_z) /
        (deno1 * deno2);
    //std::cout << cos_ang<< std::endl;
    if (cos_ang < std::cos(15 * M_PI / 180)) //��ƽ��н���ֵ
    {

        return false;
    }
    // Second, check distance between two planes
    float A = p1.normal_x / deno1, B = p1.normal_y / deno1, C = p1.normal_z / deno1;
    float param[3] = {std::abs(A), std::abs(B), std::abs(C)};
    int pos_max = std::max_element(param, param + 3) - param;

    float param1[3] = {p1.normal_x, p1.normal_y, p1.normal_z};
    float param2[3] = {p2.normal_x, p2.normal_y, p2.normal_z};
    float dist = 0.0;
    if (param1[pos_max] * param2[pos_max] > 0) {
        dist = std::abs((p1.Distance - p2.Distance) / deno1) / std::sqrt(A * A + B * B + C * C);
    } else {
        dist = std::abs((p1.Distance + p2.Distance) / deno1) / std::sqrt(A * A + B * B + C * C);
    }

    if (dist > 0.1) //��ƽ�������ֵ0.1��
    {

        return false;
    }

    // Third, check the distance of two patch
    const CloudPtr cloud_plane(new Cloud);
    for (vector<int>::iterator iter = p1.PointID.begin(), end = p1.PointID.end(); iter < end; ++iter) {
        CloudItem tempt;
        tempt.x = cloud->points.at(*iter).x;
        tempt.y = cloud->points.at(*iter).y;
        tempt.z = cloud->points.at(*iter).z;
        cloud_plane->points.push_back(tempt);
    }
    if (cloud_plane->points.size() == 0) {
        return false;
    }
    KdTreeFLANN<CloudItem> kd_tree;
    //CLOUD_BLEND_DOUBLE_NAMESPACE::KdTree kd_tree;
    std::vector<int> pts_search_idx;
    std::vector<double> pts_search_dist;
    //std::vector<CloudItem> k_points;
    kd_tree.setInputCloud(cloud_plane);

    //int num_judge = 0;
    for (vector<int>::iterator iter = p2.PointID.begin(), end = p2.PointID.end(); iter < end; ++iter) {
        CloudItem search_point;
        search_point.x = cloud->points.at(*iter).x;
        search_point.y = cloud->points.at(*iter).y;
        search_point.z = cloud->points.at(*iter).z;
        kd_tree.nearestKSearch(search_point, 1, pts_search_idx, pts_search_dist);
        //kd_tree.nearestKSearch(search_point, 1, k_points);
        if (pts_search_dist.size() == 0) {
            continue;
        }
        //��Ⱥ�������ֵ0.75
        if (pts_search_dist[0] < 2) {
            //if (PointCloudHelper::point_dis2(search_point, k_points[0]) < 2) {
            //std::cout << "Coplanar!" << std::endl;
            return true;
        }
        //++num_judge;
    }

    return false;
}

void TreeExtration::count_pca_value(PlanSegment& plane, const CloudPtr& cloud)
{
    CalculateFeature calculater;
    CloudPtr tempcloud(new Cloud);
    for (vector<int>::iterator iter = plane.PointID.begin(); iter < plane.PointID.end(); ++iter) {
        CloudItem temppt;
        tempcloud->push_back(cloud->points.at(*iter));
    }
    PlanSegment tempSegment = calculater.calculate_plan_parameter_h_points(tempcloud);
    plane.normal_x = tempSegment.normal_x;
    plane.normal_y = tempSegment.normal_y;
    plane.normal_z = tempSegment.normal_z;
    plane.Distance = tempSegment.Distance;
    //std::cout<<plane.normal_x<<plane.normal_y<<plane.normal_z<<plane.Distance<<std::endl;
}

void TreeExtration::merge_plane(PlanSegment& p1, PlanSegment& p2, const CloudPtr& cloud)
{
    for (int i = 0; i < p2.PointID.size(); i++) {
        p1.PointID.push_back(p2.PointID[i]);
    }
    //	count_pca_value(p1,cloud);    //����ϲ�����Ƭ����
}
CLOUD_BLEND_DOUBLE_NAMESPACE_END