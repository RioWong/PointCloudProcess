#pragma once
#include "cmm_types.h"
#include <Eigen/Eigen>
#include <boost/unordered_map.hpp>
#include "cloud_stamp_rot.h"
#include "concatenate.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

class PointCloudHelper
{
public:
    PointCloudHelper();
    ~PointCloudHelper();

    static void remove_duplicate(CloudPtr cloudSrc, const float voxel_grid_size);
    static void remove_duplicate(CloudPtr cloudSrc, const float voxel_grid_size, int min_num);
    static float get_rot_icp(CloudPtr cloud_src, CloudPtr cloud_temp, Eigen::Matrix4d& mat_rot, bool do_scale = false, bool do_affine = false);
    static void change_cloud_rgb(CloudPtr cloud_src, int r, int g, int b);
    static double point_dis2(const PointXYZRGBA& p1, const PointXYZRGBA& p2);
    static void cloud_double2float(CloudPtr cloud_in, Cloud_FPtr cloud_out, PointXYZRGBA center);
    template <typename PointT>
    static void
    getMinMax3D(const PointCloud<PointT>& cloud, PointT& min_pt, PointT& max_pt)
    {
        Eigen::Array4d min_p, max_p;
        min_p.setConstant(std::numeric_limits<double>::max());
        max_p.setConstant(std::numeric_limits<double>::min());

        // If the data is dense, we don't need to check for NaN
        if (cloud.is_dense) {
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> pt = cloud.points[i].getArray4dMap();
                min_p = min_p.min(pt);
                max_p = max_p.max(pt);
            }
        }
        // NaN or Inf values could exist => check for them
        else {
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                // Check if the point is invalid
                if (!pcl_isfinite (cloud.points[i].x) ||
                    !pcl_isfinite (cloud.points[i].y) ||
                    !pcl_isfinite (cloud.points[i].z))
                    continue;
                const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> pt = cloud.points[i].getArray4dMap();
                min_p = min_p.min(pt);
                max_p = max_p.max(pt);
            }
        }
        min_pt.x = min_p[0];
        min_pt.y = min_p[1];
        min_pt.z = min_p[2];
        max_pt.x = max_p[0];
        max_pt.y = max_p[1];
        max_pt.z = max_p[2];
    }

    template <typename PointT>
    inline static void
    getMinMax3D(const PointCloud<PointT>& cloud, Eigen::Vector4d& min_pt, Eigen::Vector4d& max_pt)
    {
        Eigen::Array4d min_p, max_p;
        min_p.setConstant(std::numeric_limits<double>::max());
        max_p.setConstant(std::numeric_limits<double>::min());

        // If the data is dense, we don't need to check for NaN
        if (cloud.is_dense) {
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> pt = cloud.points[i].getArray4dMap();
                min_p = min_p.min(pt);
                max_p = max_p.max(pt);
            }
        }
        // NaN or Inf values could exist => check for them
        else {
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                // Check if the point is invalid
                if (!pcl_isfinite (cloud.points[i].x) ||
                    !pcl_isfinite (cloud.points[i].y) ||
                    !pcl_isfinite (cloud.points[i].z))
                    continue;
                const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> pt = cloud.points[i].getArray4dMap();
                min_p = min_p.min(pt);
                max_p = max_p.max(pt);
            }
        }
        min_pt = min_p;
        max_pt = max_p;
    }

    template <typename PointT>
    static void
    transformPointCloud(const PointCloud<PointT>& cloud_in,
                        PointCloud<PointT>& cloud_out,
                        const Eigen::Matrix4d& transform)
    {
        if (&cloud_in != &cloud_out) {
            // Note: could be replaced by cloud_out = cloud_in
            cloud_out.header = cloud_in.header;
            cloud_out.width = cloud_in.width;
            cloud_out.height = cloud_in.height;
            cloud_out.is_dense = cloud_in.is_dense;
            cloud_out.points.reserve(cloud_out.points.size());
            cloud_out.points.assign(cloud_in.points.begin(), cloud_in.points.end());
        }

        Eigen::Matrix3d rot = transform.block<3, 3>(0, 0);
        Eigen::Vector3d trans = transform.block<3, 1>(0, 3);
        // If the data is dense, we don't need to check for NaN
        if (cloud_in.is_dense) {
            for (size_t i = 0; i < cloud_out.points.size(); ++i)
                cloud_out.points[i].getVector3dMap() = rot *
                    cloud_in.points[i].getVector3dMap() + trans;
        }
        // Dataset might contain NaNs and Infs, so check for them first.
        else {
            for (size_t i = 0; i < cloud_out.points.size(); ++i) {
                if (!pcl_isfinite (cloud_in.points[i].x) ||
                    !pcl_isfinite (cloud_in.points[i].y) ||
                    !pcl_isfinite (cloud_in.points[i].z))
                    continue;
                cloud_out.points[i].getVector3dMap() = rot *
                    cloud_in.points[i].getVector3dMap() + trans;
            }
        }
    }

    template <typename PointT>
    static void
    removeNaNFromPointCloud(const PointCloud<PointT>& cloud_in, PointCloud<PointT>& cloud_out,
                            std::vector<int>& index)
    {
        // If the clouds are not the same, prepare the output
        if (&cloud_in != &cloud_out) {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }
        // Reserve enough space for the indices
        index.resize(cloud_in.points.size());
        size_t j = 0;

        // If the data is dense, we don't need to check for NaN
        if (cloud_in.is_dense) {
            // Simply copy the data
            cloud_out = cloud_in;
            for (j = 0; j < cloud_out.points.size(); ++j)
                index[j] = static_cast<int>(j);
        } else {
            for (size_t i = 0; i < cloud_in.points.size(); ++i) {
                if (!pcl_isfinite (cloud_in.points[i].x) ||
                    !pcl_isfinite (cloud_in.points[i].y) ||
                    !pcl_isfinite (cloud_in.points[i].z))
                    continue;
                cloud_out.points[j] = cloud_in.points[i];
                index[j] = static_cast<int>(i);
                j++;
            }
            if (j != cloud_in.points.size()) {
                // Resize to the correct size
                cloud_out.points.resize(j);
                index.resize(j);
                cloud_out.height = 1;
                cloud_out.width = static_cast<uint32_t>(j);
            }
            // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
            cloud_out.is_dense = true;
        }
    }

    template <typename PointInT, typename PointOutT>
    static void
    copyPointCloud (const PointCloud<PointInT> &cloud_in, PointCloud<PointOutT> &cloud_out)
    {
        // Allocate enough space and copy the basics
        cloud_out.points.resize (cloud_in.points.size ());
        cloud_out.header   = cloud_in.header;
        cloud_out.width    = cloud_in.width;
        cloud_out.height   = cloud_in.height;
        cloud_out.is_dense = cloud_in.is_dense;
        // Copy all the data fields from the input cloud to the output one
        typedef typename traits::fieldList<PointInT>::type FieldListInT;
        typedef typename traits::fieldList<PointOutT>::type FieldListOutT;
        typedef typename intersect<FieldListInT, FieldListOutT>::type FieldList; 
        // Iterate over each point
        for (size_t i = 0; i < cloud_in.points.size (); ++i)
        {
            // Iterate over each dimension
            for_each_type <FieldList> (NdConcatenateFunctor <PointInT, PointOutT> (cloud_in.points[i], cloud_out.points[i]));
        }
    }

    template <typename PointT>
    inline static unsigned int
    compute3DCentroid(const PointCloud<PointT>& cloud, Eigen::Vector4d& centroid)
    {
        if (cloud.points.empty())
            return (0);

        // Initialize to 0
        centroid.setZero();
        // For each point in the cloud
        // If the data is dense, we don't need to check for NaN
        if (cloud.is_dense) {
            for (size_t i = 0; i < cloud.points.size(); ++i)
                centroid += cloud.points[i].getVector4dMap();
            centroid[3] = 0;
            centroid /= static_cast<double>(cloud.points.size());

            return (static_cast<unsigned int>(cloud.points.size()));
        }
        // NaN or Inf values could exist => check for them
        else {
            unsigned cp = 0;
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                // Check if the point is invalid
                if (!pcl_isfinite (cloud.points[i].x) ||
                    !pcl_isfinite (cloud.points[i].y) ||
                    !pcl_isfinite (cloud.points[i].z))
                    continue;

                centroid += cloud[i].getVector4dMap();
                ++cp;
            }
            centroid[3] = 0;
            centroid /= static_cast<double>(cp);

            return (cp);
        }
    }
    static void cloud_double2float(CloudPtr cloud_in, Cloud_FPtr cloud_out);
    static void dpcd2fpcd(std::string str_path);

private:
    static double point2plane_dis(double x0, double y0, double z0);
    static double point2origin_dis(double x0, double y0, double z0);
    DISALLOW_COPY_AND_ASSIGN(PointCloudHelper);
};
CLOUD_BLEND_DOUBLE_NAMESPACE_END
