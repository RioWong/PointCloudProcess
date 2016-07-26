#pragma once
#include "cmm_types.h"
#include <vector>
#include <ostream>
#include "point_type.h"
#include "point_cloud.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
//using namespace std;
/*las��ʽ�����ݸ�ʽ��֧��LAS1.0 1.1 1.2*/
//�ļ�ͷ227���ֽ�//
struct HEADERBLOCK
{
    char FileSignature[4];
    unsigned long Reserved;
    unsigned long ProjectIDGUIDdata1;
    unsigned short ProjectIDGUIDdata2;
    unsigned short ProjectIDGUIDdata3;
    unsigned char ProjectIDGUIDdata4[8];
    unsigned char VersionMajor;//���汾��1
    unsigned char VersionMinor;//���汾��.0
    char SystemIdentifier[32];
    char GeneratingSoftware[32];
    unsigned short FileCreationDayofYear;
    unsigned short FileCreationYear;
    unsigned short HeaderSize;//�ļ�ͷ�Ĵ�С
    unsigned long OffSetToPointData;//���ļ���ͷ�������ݼ�¼ƫ�Ƶ��ֽ���
    unsigned long NumberOfVariableLengthRecords;//�䳤��¼�ĸ���
    unsigned char PointDataFormatID;//�����ݼ�¼�ĸ�ʽ��
    unsigned short PointDataRecordLength;//ÿ�����¼���ֽ���
    unsigned long NumberOfPointRecords;//This field contains the total number of point records within the file.
    unsigned long Numberofpointsbyreturn[5];/* This field contains an array of the total point records per return.  
The first unsigned long value will be the total number of records from the first return, and the 
  second contains the total number for return two, and so forth up to five returns. */

    double XScaleFactor;//x�ı����߻�λ
    double YScaleFactor;//y�ı����߻�λ
    double ZScaleFactor;//z�ı����߻�λ
    double XOffSet;
    double YOffSet;
    double ZOffSet;
    double MaxX;
    double MinX;
    double MaxY;
    double MinY;
    double MaxZ;
    double MinZ;
};

//�䳤��¼ͷ54 ���ֽ�//
struct VARISBLELENGTHRECORDS
{
    unsigned short RecordSignature;
    char UserID[16];
    unsigned short RecordID;
    unsigned short RecordLengthAfterHeader;//�䳤��¼�ڱ߳���¼ͷ����ֽ���
    char Description[32];
};

/*���¼�ĸ�ʽ0*/
struct POINTDATARECORDFORMAT0
{
    /*The X, Y, and Z values are stored as long integers.  The X, Y, and Z values are 
    used in conjunction with the scale values and the offset values to determine the 
    coordinate for each point.
    Xcoordinate = (Xrecord * Xscale) + Xoffset 
    Ycoordinate = (Yrecord * Yscale) + Yoffset 
    Zcoordinate = (Zrecord * Zscale) + Zoffset */
    long X;//
    long Y;//
    long Z;//
    unsigned short Intensity;//����ǿ��
    unsigned short ReturnNumber;//�ܻز�����
    unsigned short NumberofReturns;//���λز���
    unsigned short ScanDirectionFlag;/*A bit value of 1 is a positive scan direction, and a 
bit value of 0 is a negative scan direction (where positive scan direction is a scan moving 
 from the left side of the in-track direction to the right side and negative the opposite). */
    unsigned short EdgeofFlightLine;/*The Edge of Flight Line data bit has a value of 1 only when the point is at 
the end of a scan.  It is the last point on a given scan line before it changes direction. */
    unsigned char Classification;//
    char ScanAngleRank;/*The Scan Angle Rank is a signed one-byte number with a valid range from -90 
                to +90.  The Scan Angle Rank is the angle (rounded to the nearest integer in the absolute value 
                sense) at which the laser point was output from the laser system including the roll of the aircraft.  
                The scan angle is within 1 degree of accuracy from +90 to �C90 degrees.  The scan angle is an 
                angle based on 0 degrees being nadir, and �C90 degrees to the left side of the aircraft in the 
                direction of flight.*/
    unsigned char UserData;
    unsigned short PointSourceID;
};

/*���¼�ĸ�ʽ1*/
struct POINTDATARECORDFORMAT1
{
    long X;
    long Y;
    long Z;
    unsigned short Intensity;
    unsigned short ReturnNumber;
    unsigned short NumberofReturns;
    unsigned short ScanDirectionFlag;
    unsigned short EdgeofFlightLine;
    unsigned char Classification;
    char ScanAngleRank;
    unsigned char UserData;
    unsigned short PointSourceID;
    double GPSTime;//gpsʱ��
};

struct POINTDATARECORDFORMAT2
{
    long X;
    long Y;
    long Z;
    unsigned short Intensity;
    unsigned short ReturnNumber;
    unsigned short NumberofReturns;
    unsigned short ScanDirectionFlag;
    unsigned short EdgeofFlightLine;
    unsigned char Classification;
    char ScanAngleRank;
    unsigned char UserData;
    unsigned short PointSourceID;
    unsigned short R;
    unsigned short G;
    unsigned short B;
};

struct POINTDATARECORDFORMAT3
{
    long X;
    long Y;
    long Z;
    unsigned short Intensity;
    unsigned short ReturnNumber;
    unsigned short NumberofReturns;
    unsigned short ScanDirectionFlag;
    unsigned short EdgeofFlightLine;
    unsigned char Classification;
    char ScanAngleRank;
    unsigned char UserData;
    unsigned short PointSourceID;
    unsigned short R;
    unsigned short G;
    unsigned short B;
    double GPSTime;//gpsʱ��
};


//�Զ����LiDAR����x,y,z,R,G,B���;
struct LAS_POINT
{
    double x;
    double y;
    float z;
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

//LiDAR�������
struct LAS_POINT_PROPERTY
{
    /*SuperVoxel�ķ�����;*/
    float normal_x; //ƽ�淽�̲���
    float normal_y;
    float normal_z;
    double Distance;
    double curvature; //����
    int PointID; //���
    int SegmentID;
    float dis_from_point_plane;
};

struct PROPERTY
{
    int PointID;
    float dis_from_point_plane;
};

struct LAS_POINT_PROPERTY_sim
{
    float x;
    float y;
    float z;
    float dis_from_point_plane;
};

struct PlanSegment
{
    unsigned short SegmentID;
    std::vector<int> PointID;
    float normal_x;
    float normal_y;
    float normal_z;
    float min_value; //lamda3
    float curvature; //����
    float Distance;
};

struct PolarPoint
{
    double Z; //����
    double H; //ˮƽ��
    double R; //����
};

struct XYZPoint
{
    double X, Y, Z; //����
};

struct angeldfm
{
    int du, fen, miao;
};


//typedef PointXYZRGBA CloudItem;//XYZRGBA��ʽ�ĵ��Ƶ�Ԫ
//typedef PointCloud< CloudItem> Cloud;//��XYZRGBA��ʽ�ĵ��Ƶ�Ԫ���ɵĵ�������
//typedef Cloud::ConstPtr CloudConstPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶĳ���ָ��
//typedef Cloud::Ptr CloudPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶ�ָ��,����pcl��PtrΪshared_ptr����

CLOUD_BLEND_DOUBLE_NAMESPACE_END
