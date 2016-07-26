#pragma once
#include "cmm_types.h"
#include <vector>
#include <ostream>
#include "point_type.h"
#include "point_cloud.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
//using namespace std;
/*las格式的数据格式，支持LAS1.0 1.1 1.2*/
//文件头227个字节//
struct HEADERBLOCK
{
    char FileSignature[4];
    unsigned long Reserved;
    unsigned long ProjectIDGUIDdata1;
    unsigned short ProjectIDGUIDdata2;
    unsigned short ProjectIDGUIDdata3;
    unsigned char ProjectIDGUIDdata4[8];
    unsigned char VersionMajor;//主版本号1
    unsigned char VersionMinor;//副版本号.0
    char SystemIdentifier[32];
    char GeneratingSoftware[32];
    unsigned short FileCreationDayofYear;
    unsigned short FileCreationYear;
    unsigned short HeaderSize;//文件头的大小
    unsigned long OffSetToPointData;//从文件开头到点数据记录偏移的字节数
    unsigned long NumberOfVariableLengthRecords;//变长记录的个数
    unsigned char PointDataFormatID;//点数据记录的格式号
    unsigned short PointDataRecordLength;//每个点记录的字节数
    unsigned long NumberOfPointRecords;//This field contains the total number of point records within the file.
    unsigned long Numberofpointsbyreturn[5];/* This field contains an array of the total point records per return.  
The first unsigned long value will be the total number of records from the first return, and the 
  second contains the total number for return two, and so forth up to five returns. */

    double XScaleFactor;//x的比例尺或单位
    double YScaleFactor;//y的比例尺或单位
    double ZScaleFactor;//z的比例尺或单位
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

//变长记录头54 个字节//
struct VARISBLELENGTHRECORDS
{
    unsigned short RecordSignature;
    char UserID[16];
    unsigned short RecordID;
    unsigned short RecordLengthAfterHeader;//变长记录在边长记录头后的字节数
    char Description[32];
};

/*点记录的格式0*/
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
    unsigned short Intensity;//反射强度
    unsigned short ReturnNumber;//总回波次数
    unsigned short NumberofReturns;//本次回波数
    unsigned short ScanDirectionFlag;/*A bit value of 1 is a positive scan direction, and a 
bit value of 0 is a negative scan direction (where positive scan direction is a scan moving 
 from the left side of the in-track direction to the right side and negative the opposite). */
    unsigned short EdgeofFlightLine;/*The Edge of Flight Line data bit has a value of 1 only when the point is at 
the end of a scan.  It is the last point on a given scan line before it changes direction. */
    unsigned char Classification;//
    char ScanAngleRank;/*The Scan Angle Rank is a signed one-byte number with a valid range from -90 
                to +90.  The Scan Angle Rank is the angle (rounded to the nearest integer in the absolute value 
                sense) at which the laser point was output from the laser system including the roll of the aircraft.  
                The scan angle is within 1 degree of accuracy from +90 to –90 degrees.  The scan angle is an 
                angle based on 0 degrees being nadir, and –90 degrees to the left side of the aircraft in the 
                direction of flight.*/
    unsigned char UserData;
    unsigned short PointSourceID;
};

/*点记录的格式1*/
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
    double GPSTime;//gps时间
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
    double GPSTime;//gps时间
};


//自定义的LiDAR点由x,y,z,R,G,B组成;
struct LAS_POINT
{
    double x;
    double y;
    float z;
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

//LiDAR点的属性
struct LAS_POINT_PROPERTY
{
    /*SuperVoxel的法向量;*/
    float normal_x; //平面方程参数
    float normal_y;
    float normal_z;
    double Distance;
    double curvature; //曲率
    int PointID; //点号
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
    float curvature; //曲率
    float Distance;
};

struct PolarPoint
{
    double Z; //仰角
    double H; //水平角
    double R; //距离
};

struct XYZPoint
{
    double X, Y, Z; //坐标
};

struct angeldfm
{
    int du, fen, miao;
};


//typedef PointXYZRGBA CloudItem;//XYZRGBA格式的点云单元
//typedef PointCloud< CloudItem> Cloud;//由XYZRGBA格式的点云单元构成的点云数据
//typedef Cloud::ConstPtr CloudConstPtr;//指向带有颜色信息的点云的常量指针
//typedef Cloud::Ptr CloudPtr;//指向带有颜色信息的点云的指针,点云pcl中Ptr为shared_ptr类型

CLOUD_BLEND_DOUBLE_NAMESPACE_END
