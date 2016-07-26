#pragma once
#include <boost/type_traits/is_same.hpp>
#include "macros.h"
#include "point_cloud_helper.h"
#include "exception.h"
#include "point_traits.h"
#include "sensor_point_cloud2.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
inline int
getFieldIndex(const sensor_msgs::PointCloud2& cloud, const std::string& field_name)
{
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size(); ++d)
        if (cloud.fields[d].name == field_name)
            return (static_cast<int>(d));
    return (-1);
}

template <typename PointT>
int
getFieldIndex(const PointCloud<PointT>&,
              const std::string& field_name,
              std::vector<sensor_msgs::PointField>& fields)
{
    fields.clear();
    // Get the fields list
    for_each_type<typename traits::fieldList<PointT>::type>(detail::FieldAdder<PointT>(fields));
    for (size_t d = 0; d < fields.size(); ++d)
        if (fields[d].name == field_name)
            return (static_cast<int>(d));
    return (-1);
}

template <typename PointT>
int
getFieldIndex(const std::string& field_name,
              std::vector<sensor_msgs::PointField>& fields)
{
    fields.clear();
    // Get the fields list
    for_each_type<typename traits::fieldList<PointT>::type>(detail::FieldAdder<PointT>(fields));
    for (size_t d = 0; d < fields.size(); ++d)
        if (fields[d].name == field_name)
            return (static_cast<int>(d));
    return (-1);
}
struct RGB
{
    union
    {
        union
        {
            struct
            {
                uint8_t b;
                uint8_t g;
                uint8_t r;
                uint8_t a;
            };

            float rgb;
        };

        uint32_t rgba;
    };
};

struct cloud_point_index_idx
{
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_)
    {
    }

    bool operator <(const cloud_point_index_idx& p) const
    {
        return (idx < p.idx);
    }
};

struct PointIndices
{
    PointIndices() : header(), indices()
    {
    }

    Header header;

    std::vector<int> indices;

public:
    typedef boost::shared_ptr<PointIndices> Ptr;
    typedef boost::shared_ptr<PointIndices const> ConstPtr;
}; // struct PointIndices

// definitions used everywhere
typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
getMinMax3D (const typename PointCloud<PointT>::ConstPtr &cloud,
                  const std::string &distance_field_name, double min_distance, double max_distance,
                  Eigen::Vector4d &min_pt, Eigen::Vector4d &max_pt, bool limit_negative)
{
  Eigen::Array4d min_p, max_p;
  min_p.setConstant (std::numeric_limits<double>::max());
  max_p.setConstant (std::numeric_limits<double>::min());

  // Get the fields list and the distance field index
  std::vector<sensor_msgs::PointField> fields;
  int distance_idx = getFieldIndex (*cloud, distance_field_name, fields);

  double distance_value;
  // If dense, no need to check for NaNs
  if (cloud->is_dense)
  {
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud->points[i]);
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (double));

      if (limit_negative)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < max_distance) && (distance_value > min_distance))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > max_distance) || (distance_value < min_distance))
          continue;
      }
      // Create the point structure and get the min/max
      const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> pt = cloud->points[i].getArray4dMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  else
  {
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud->points[i]);
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (double));

      if (limit_negative)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < max_distance) && (distance_value > min_distance))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > max_distance) || (distance_value < min_distance))
          continue;
      }

      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) || 
          !pcl_isfinite (cloud->points[i].y) || 
          !pcl_isfinite (cloud->points[i].z))
        continue;
      // Create the point structure and get the min/max
      const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> pt = cloud->points[i].getArray4dMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt = min_p;
  max_pt = max_p;
}

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief PCL base class. Implements methods that are used by all PCL objects. 
* \ingroup common 
*/
template <typename PointT>
class PCLBase
{
public:
    typedef CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef PointIndices::Ptr PointIndicesPtr;
    typedef PointIndices::ConstPtr PointIndicesConstPtr;

    /** \brief Empty constructor. */
    PCLBase() : input_(), indices_(), use_indices_(false), fake_indices_(false)
    {
    }

    /** \brief Copy constructor. */
    PCLBase(const PCLBase& base)
        : input_(base.input_)
          , indices_(base.indices_)
          , use_indices_(base.use_indices_)
          , fake_indices_(base.fake_indices_)
    {
    }

    /** \brief destructor. */
    virtual ~PCLBase()
    {
        input_.reset();
        indices_.reset();
    }

    /** \brief Provide a pointer to the input dataset
    * \param cloud the const boost shared pointer to a PointCloud message
    */
    virtual inline void
    setInputCloud(const PointCloudConstPtr& cloud)
    {
        input_ = cloud;
    }

    /** \brief Get a pointer to the input point cloud dataset. */
    inline PointCloudConstPtr const
    getInputCloud()
    {
        return (input_);
    }

    /** \brief Provide a pointer to the vector of indices that represents the input data.
    * \param indices a pointer to the vector of indices that represents the input data.
    */
    inline void
    setIndices(const IndicesPtr& indices)
    {
        indices_ = indices;
        fake_indices_ = false;
        use_indices_ = true;
    }

    /** \brief Provide a pointer to the vector of indices that represents the input data.
    * \param indices a pointer to the vector of indices that represents the input data.
    */
    inline void
    setIndices(const IndicesConstPtr& indices)
    {
        indices_.reset(new std::vector<int>(*indices));
        fake_indices_ = false;
        use_indices_ = true;
    }

    /** \brief Provide a pointer to the vector of indices that represents the input data.
    * \param indices a pointer to the vector of indices that represents the input data.
    */
    inline void
    setIndices(const PointIndicesConstPtr& indices)
    {
        indices_.reset(new std::vector<int>(indices->indices));
        fake_indices_ = false;
        use_indices_ = true;
    }

    /** \brief Set the indices for the points laying within an interest region of 
    * the point cloud.
    * \note you shouldn't call this method on unorganized point clouds!
    * \param row_start the offset on rows
    * \param col_start the offset on columns
    * \param nb_rows the number of rows to be considered row_start included
    * \param nb_cols the number of columns to be considered col_start included
    */
    inline void
    setIndices(size_t row_start, size_t col_start, size_t nb_rows, size_t nb_cols)
    {
        if ((nb_rows > input_->height) || (row_start > input_->height)) {
            PCL_ERROR("[PCLBase::setIndices] cloud is only %d height", input_->height);
            return;
        }

        if ((nb_cols > input_->width) || (col_start > input_->width)) {
            PCL_ERROR("[PCLBase::setIndices] cloud is only %d width", input_->width);
            return;
        }

        size_t row_end = row_start + nb_rows;
        if (row_end > input_->height) {
            PCL_ERROR("[PCLBase::setIndices] %d is out of rows range %d", row_end, input_->height);
            return;
        }

        size_t col_end = col_start + nb_cols;
        if (col_end > input_->width) {
            PCL_ERROR("[PCLBase::setIndices] %d is out of columns range %d", col_end, input_->width);
            return;
        }

        indices_.reset(new std::vector<int>);
        indices_->reserve(nb_cols * nb_rows);
        for (size_t i = row_start; i < row_end; i++)
            for (size_t j = col_start; j < col_end; j++)
                indices_->push_back(static_cast<int>((i * input_->width) + j));
        fake_indices_ = false;
        use_indices_ = true;
    }

    /** \brief Get a pointer to the vector of indices used. */
    inline IndicesPtr const
    getIndices()
    {
        return (indices_);
    }

    /** \brief Override PointCloud operator[] to shorten code
    * \note this method can be called instead of (*input_)[(*indices_)[pos]]
    * or input_->points[(*indices_)[pos]]
    * \param pos position in indices_ vector
    */
    const PointT& operator[](size_t pos)
    {
        return ((*input_)[(*indices_)[pos]]);
    }

protected:
    /** \brief The input point cloud dataset. */
    PointCloudConstPtr input_;

    /** \brief A pointer to the vector of point indices to use. */
    IndicesPtr indices_;

    /** \brief Set to true if point indices are used. */
    bool use_indices_;

    /** \brief If no set of indices are given, we construct a set of fake indices that mimic the input PointCloud. */
    bool fake_indices_;

    /** \brief This method should get called before starting the actual computation. 
    *
    * Internally, initCompute() does the following:
    *   - checks if an input dataset is given, and returns false otherwise
    *   - checks whether a set of input indices has been given. Returns true if yes.
    *   - if no input indices have been given, a fake set is created, which will be used until:
    *     - either a new set is given via setIndices(), or 
    *     - a new cloud is given that has a different set of points. This will trigger an update on the set of fake indices
    */
    inline bool
    initCompute()
    {
        // Check if input was set
        if (!input_)
            return (false);

        // If no point indices have been given, construct a set of indices for the entire input point cloud
        if (!indices_) {
            fake_indices_ = true;
            indices_.reset(new std::vector<int>);
            try {
                indices_->resize(input_->points.size());
            }
            catch (std::bad_alloc) {
                //PCL_ERROR ("[initCompute] Failed to allocate %zu indices.\n", input_->points.size ());
            }
            for (size_t i = 0; i < indices_->size(); ++i) {
                (*indices_)[i] = static_cast<int>(i);
            }
        }

        // If we have a set of fake indices, but they do not match the number of points in the cloud, update them
        if (fake_indices_ && indices_->size() != input_->points.size()) {
            size_t indices_size = indices_->size();
            indices_->resize(input_->points.size());
            for (size_t i = indices_size; i < indices_->size(); ++i) {
                (*indices_)[i] = static_cast<int>(i);
            }
        }

        return (true);
    }

    /** \brief This method should get called after finishing the actual computation. 
    *
    */
    inline bool
    deinitCompute()
    {
        return (true);
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointT>
class Filter : public PCLBase<PointT>
{
public:
    using PCLBase<PointT>::indices_;
    using PCLBase<PointT>::input_;

    typedef boost::shared_ptr<Filter<PointT> > Ptr;
    typedef boost::shared_ptr<const Filter<PointT> > ConstPtr;

    typedef CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    /** \brief Empty constructor.
    * \param[in] extract_removed_indices set to true if the filtered data indices should be saved in a 
    * separate list. Default: false.
    */
    Filter(bool extract_removed_indices = false) :
        removed_indices_(new std::vector<int>),
        filter_name_(),
        extract_removed_indices_(extract_removed_indices)
    {
    }

    /** \brief Get the point indices being removed */
    inline IndicesConstPtr const
    getRemovedIndices()
    {
        return (removed_indices_);
    }

    /** \brief Calls the filtering method and returns the filtered dataset in output.
    * \param[out] output the resultant filtered point cloud dataset
    */
    inline void
    filter(PointCloud& output)
    {
        if (!initCompute())
            return;

        // Resize the output dataset
        //if (output.points.size () != indices_->size ())
        //  output.points.resize (indices_->size ());

        // Copy header at a minimum
        output.header = input_->header;
        output.sensor_origin_ = input_->sensor_origin_;
        output.sensor_orientation_ = input_->sensor_orientation_;

        // Apply the actual filter
        applyFilter(output);

        deinitCompute();
    }

protected:

    using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

    /** \brief Indices of the points that are removed */
    IndicesPtr removed_indices_;

    /** \brief The filter name. */
    std::string filter_name_;

    /** \brief Set to true if we want to return the indices of the removed points. */
    bool extract_removed_indices_;

    /** \brief Abstract filter method. 
    *
    * The implementation needs to set output.{points, width, height, is_dense}.
    *
    * \param[out] output the resultant filtered point cloud
    */
    virtual void
    applyFilter(PointCloud& output) = 0;

    /** \brief Get a string representation of the name of this class. */
    inline const std::string&
    getClassName() const
    {
        return (filter_name_);
    }
};

template <typename PointT>
class VoxelGrid: public Filter<PointT>
{
protected:
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

public:
    /** \brief Empty constructor. */
    VoxelGrid() :
        leaf_size_(Eigen::Vector4d::Zero()),
        inverse_leaf_size_(Eigen::Array4d::Zero()),
        downsample_all_data_(true),
        save_leaf_layout_(false),
        leaf_layout_(),
        min_b_(Eigen::Vector4i::Zero()),
        max_b_(Eigen::Vector4i::Zero()),
        div_b_(Eigen::Vector4i::Zero()),
        divb_mul_(Eigen::Vector4i::Zero()),
        filter_field_name_(""),
        filter_limit_min_(-FLT_MAX),
        filter_limit_max_(FLT_MAX),
        filter_limit_negative_(false)
    {
        filter_name_ = "VoxelGrid";
    }

    /** \brief Destructor. */
    virtual ~VoxelGrid()
    {
    }

    /** \brief Set the voxel grid leaf size.
      * \param[in] leaf_size the voxel grid leaf size
      */
    inline void
    setLeafSize(const Eigen::Vector4d& leaf_size)
    {
        leaf_size_ = leaf_size;
        // Avoid division errors
        if (leaf_size_[3] == 0)
            leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4d::Ones() / leaf_size_.array();
    }

    /** \brief Set the voxel grid leaf size.
      * \param[in] lx the leaf size for X
      * \param[in] ly the leaf size for Y
      * \param[in] lz the leaf size for Z
      */
    inline void
    setLeafSize(double lx, double ly, double lz)
    {
        leaf_size_[0] = lx;
        leaf_size_[1] = ly;
        leaf_size_[2] = lz;
        // Avoid division errors
        if (leaf_size_[3] == 0)
            leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4d::Ones() / leaf_size_.array();
    }

    /** \brief Get the voxel grid leaf size. */
    inline Eigen::Vector3d
    getLeafSize()
    {
        return (leaf_size_.head<3>());
    }

    /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
      * \param[in] downsample the new value (true/false)
      */
    inline void
    setDownsampleAllData(bool downsample)
    {
        downsample_all_data_ = downsample;
    }

    /** \brief Get the state of the internal downsampling parameter (true if
      * all fields need to be downsampled, false if just XYZ). 
      */
    inline bool
    getDownsampleAllData()
    {
        return (downsample_all_data_);
    }

    /** \brief Set to true if leaf layout information needs to be saved for later access.
      * \param[in] save_leaf_layout the new value (true/false)
      */
    inline void
    setSaveLeafLayout(bool save_leaf_layout)
    {
        save_leaf_layout_ = save_leaf_layout;
    }

    /** \brief Returns true if leaf layout information will to be saved for later access. */
    inline bool
    getSaveLeafLayout()
    {
        return (save_leaf_layout_);
    }

    /** \brief Get the minimum coordinates of the bounding box (after
      * filtering is performed). 
      */
    inline Eigen::Vector3i
    getMinBoxCoordinates()
    {
        return (min_b_.head<3>());
    }

    /** \brief Get the minimum coordinates of the bounding box (after
      * filtering is performed). 
      */
    inline Eigen::Vector3i
    getMaxBoxCoordinates()
    {
        return (max_b_.head<3>());
    }

    /** \brief Get the number of divisions along all 3 axes (after filtering
      * is performed). 
      */
    inline Eigen::Vector3i
    getNrDivisions()
    {
        return (div_b_.head<3>());
    }

    /** \brief Get the multipliers to be applied to the grid coordinates in
      * order to find the centroid index (after filtering is performed). 
      */
    inline Eigen::Vector3i
    getDivisionMultiplier()
    {
        return (divb_mul_.head<3>());
    }

    /** \brief Returns the index in the resulting downsampled cloud of the specified point.
      *
      * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering 
      * performed, and that the point is inside the grid, to avoid invalid access (or use
      * getGridCoordinates+getCentroidIndexAt)
      *
      * \param[in] p the point to get the index at
      */
    inline int
    getCentroidIndex(const PointT& p)
    {
        return (leaf_layout_.at((Eigen::Vector4i(static_cast<int>(double(p.x * inverse_leaf_size_[0])),
                                                 static_cast<int>(double(p.y * inverse_leaf_size_[1])),
                                                 static_cast<int>(double(p.z * inverse_leaf_size_[2])), 0) - min_b_).dot(divb_mul_)));
    }

    /** \brief Returns the indices in the resulting downsampled cloud of the points at the specified grid coordinates,
      * relative to the grid coordinates of the specified point (or -1 if the cell was empty/out of bounds).
      * \param[in] reference_point the coordinates of the reference point (corresponding cell is allowed to be empty/out of bounds)
      * \param[in] relative_coordinates matrix with the columns being the coordinates of the requested cells, relative to the reference point's cell
      * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering performed
      */
    inline std::vector<int>
    getNeighborCentroidIndices(const PointT& reference_point, const Eigen::MatrixXi& relative_coordinates)
    {
        Eigen::Vector4i ijk(static_cast<int>(double(reference_point.x * inverse_leaf_size_[0])),
                            static_cast<int>(double(reference_point.y * inverse_leaf_size_[1])),
                            static_cast<int>(double(reference_point.z * inverse_leaf_size_[2])), 0);
        Eigen::Array4i diff2min = min_b_ - ijk;
        Eigen::Array4i diff2max = max_b_ - ijk;
        std::vector<int> neighbors(relative_coordinates.cols());
        for (int ni = 0; ni < relative_coordinates.cols(); ni++) {
            Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni) , 0).finished();
            // checking if the specified cell is in the grid
            if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
                neighbors[ni] = leaf_layout_[((ijk + displacement - min_b_).dot(divb_mul_))]; // .at() can be omitted
            else
                neighbors[ni] = -1; // cell is out of bounds, consider it empty
        }
        return (neighbors);
    }

    /** \brief Returns the layout of the leafs for fast access to cells relative to current position.
      * \note position at (i-min_x) + (j-min_y)*div_x + (k-min_z)*div_x*div_y holds the index of the element at coordinates (i,j,k) in the grid (-1 if empty)
      */
    inline std::vector<int>
    getLeafLayout()
    {
        return (leaf_layout_);
    }

    /** \brief Returns the corresponding (i,j,k) coordinates in the grid of point (x,y,z). 
      * \param[in] x the X point coordinate to get the (i, j, k) index at
      * \param[in] y the Y point coordinate to get the (i, j, k) index at
      * \param[in] z the Z point coordinate to get the (i, j, k) index at
      */
    inline Eigen::Vector3i
    getGridCoordinates(double x, double y, double z)
    {
        return (Eigen::Vector3i(static_cast<int>(double(x * inverse_leaf_size_[0])),
                                static_cast<int>(double(y * inverse_leaf_size_[1])),
                                static_cast<int>(double(z * inverse_leaf_size_[2]))));
    }

    /** \brief Returns the index in the downsampled cloud corresponding to a given set of coordinates.
      * \param[in] ijk the coordinates (i,j,k) in the grid (-1 if empty)
      */
    inline int
    getCentroidIndexAt(const Eigen::Vector3i& ijk)
    {
        int idx = ((Eigen::Vector4i() << ijk , 0).finished() - min_b_).dot(divb_mul_);
        if (idx < 0 || idx >= static_cast<int>(leaf_layout_.size())) // this checks also if leaf_layout_.size () == 0 i.e. everything was computed as needed
        {
            //if (verbose)
            //  PCL_ERROR ("[pcl::%s::getCentroidIndexAt] Specified coordinate is outside grid bounds, or leaf layout is not saved, make sure to call setSaveLeafLayout(true) and filter(output) first!\n", getClassName ().c_str ());
            return (-1);
        }
        return (leaf_layout_[idx]);
    }

    /** \brief Provide the name of the field to be used for filtering data. In conjunction with  \a setFilterLimits,
      * points having values outside this interval will be discarded.
      * \param[in] field_name the name of the field that contains values used for filtering
      */
    inline void
    setFilterFieldName(const std::string& field_name)
    {
        filter_field_name_ = field_name;
    }

    /** \brief Get the name of the field used for filtering. */
    inline std::string const
    getFilterFieldName()
    {
        return (filter_field_name_);
    }

    /** \brief Set the field filter limits. All points having field values outside this interval will be discarded.
      * \param[in] limit_min the minimum allowed field value
      * \param[in] limit_max the maximum allowed field value
      */
    inline void
    setFilterLimits(const double& limit_min, const double& limit_max)
    {
        filter_limit_min_ = limit_min;
        filter_limit_max_ = limit_max;
    }

    /** \brief Get the field filter limits (min/max) set by the user. The default values are -FLT_MAX, FLT_MAX. 
      * \param[out] limit_min the minimum allowed field value
      * \param[out] limit_max the maximum allowed field value
      */
    inline void
    getFilterLimits(double& limit_min, double& limit_max)
    {
        limit_min = filter_limit_min_;
        limit_max = filter_limit_max_;
    }

    /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max).
      * Default: false.
      * \param[in] limit_negative return data inside the interval (false) or outside (true)
      */
    inline void
    setFilterLimitsNegative(const bool limit_negative)
    {
        filter_limit_negative_ = limit_negative;
    }

    /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
      * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
      */
    inline void
    getFilterLimitsNegative(bool& limit_negative)
    {
        limit_negative = filter_limit_negative_;
    }

    /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
      * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
      */
    inline bool
    getFilterLimitsNegative()
    {
        return (filter_limit_negative_);
    }

protected:
    /** \brief The size of a leaf. */
    Eigen::Vector4d leaf_size_;

    /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
    Eigen::Array4d inverse_leaf_size_;

    /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
    bool downsample_all_data_;

    /** \brief Set to true if leaf layout information needs to be saved in \a leaf_layout_. */
    bool save_leaf_layout_;

    /** \brief The leaf layout information for fast access to cells relative to current position **/
    std::vector<int> leaf_layout_;

    /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

    /** \brief The desired user filter field name. */
    std::string filter_field_name_;

    /** \brief The minimum allowed filter value a point will be considered from. */
    double filter_limit_min_;

    /** \brief The maximum allowed filter value a point will be considered from. */
    double filter_limit_max_;

    /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
    bool filter_limit_negative_;

    typedef typename traits::fieldList<PointT>::type FieldList;

    /** \brief Downsample a Point Cloud using a voxelized grid approach
      * \param[out] output the resultant point cloud message
      */
    void
    applyFilter(PointCloud& output)
    {
        // Has the input dataset been set already?
        if (!input_) {
            std::cout << "[applyFilter] No input dataset given" << std::endl;
            //PCL_WARN("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
            output.width = output.height = 0;
            output.points.clear();
            return;
        }

        // Copy the header (and thus the frame_id) + allocate enough space for points
        output.height = 1; // downsampling breaks the organized structure
        output.is_dense = true; // we filter out invalid points

        Eigen::Vector4d min_p, max_p;
        // Get the minimum and maximum dimensions
        if (!filter_field_name_.empty()) // If we don't want to process the entire cloud...
            getMinMax3D<PointT>(input_, filter_field_name_, static_cast<double>(filter_limit_min_), static_cast<double>(filter_limit_max_), min_p, max_p, filter_limit_negative_);
        else
            PointCloudHelper::getMinMax3D<PointT>(*input_, min_p, max_p);

        // Compute the minimum and maximum bounding box values
        min_b_[0] = static_cast<int>(double(min_p[0] * inverse_leaf_size_[0]));
        max_b_[0] = static_cast<int>(double(max_p[0] * inverse_leaf_size_[0]));
        min_b_[1] = static_cast<int>(double(min_p[1] * inverse_leaf_size_[1]));
        max_b_[1] = static_cast<int>(double(max_p[1] * inverse_leaf_size_[1]));
        min_b_[2] = static_cast<int>(double(min_p[2] * inverse_leaf_size_[2]));
        max_b_[2] = static_cast<int>(double(max_p[2] * inverse_leaf_size_[2]));

        // Compute the number of divisions needed along all axis
        div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
        div_b_[3] = 0;

        // Set up the division multiplier
        divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

        int centroid_size = 4;
        if (downsample_all_data_) {
            //centroid_size = boost::mpl::size<FieldList>::value;
            //centroid_size = boost::mpl::size<typename traits::fieldList<PointT>::type>::value;
            centroid_size = 0;
            std::vector<sensor_msgs::PointField> fields_all;
            fields_all.clear();
            for_each_type<typename traits::fieldList<PointT>::type>(detail::FieldAdder<PointT>(fields_all));
            centroid_size = fields_all.size();
            //if (boost::is_same<PointT, cloud_blend_double::PointXYZRGBA>::value) {
            //    centroid_size = 5;
            //}
            //centroid_size = boost::mpl::size<FieldList>::value;
        }
        //std::cout << "a: " << centroid_size << std::endl;

        // ---[ RGB special case
        std::vector<sensor_msgs::PointField> fields;
        int rgba_index = -1;
        rgba_index = getFieldIndex(*input_, "rgb", fields);
        if (rgba_index == -1)
            rgba_index = getFieldIndex(*input_, "rgba", fields);
        if (rgba_index >= 0) {
            rgba_index = fields[rgba_index].offset;
            centroid_size += 3;
        }

        std::vector<cloud_point_index_idx> index_vector;
        index_vector.reserve(input_->points.size());

        // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
        if (!filter_field_name_.empty()) {
            // Get the distance field index
            std::vector<sensor_msgs::PointField> fields;
            int distance_idx = getFieldIndex(*input_, filter_field_name_, fields);
            if (distance_idx == -1)
                std::cout << "[applyFilter] Invalid filter field name. Index is " << distance_idx << "." << std::endl;
                //PCL_WARN("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName().c_str(), distance_idx);

            // First pass: go over all points and insert them into the index_vector vector
            // with calculated idx. Points with the same idx value will contribute to the
            // same point of resulting CloudPoint
            for (unsigned int cp = 0; cp < static_cast<unsigned int>(input_->points.size()); ++cp) {
                if (!input_->is_dense)
                // Check if the point is invalid
                    if (!pcl_isfinite(input_->points[cp].x) ||
                        !pcl_isfinite(input_->points[cp].y) ||
                        !pcl_isfinite(input_->points[cp].z))
                        continue;

                // Get the distance value
                const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&input_->points[cp]);
                double distance_value = 0;
                memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof (double));

                if (filter_limit_negative_) {
                    // Use a threshold for cutting out points which inside the interval
                    if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                        continue;
                } else {
                    // Use a threshold for cutting out points which are too close/far away
                    if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                        continue;
                }

                int ijk0 = static_cast<int>(double(input_->points[cp].x * inverse_leaf_size_[0]) - min_b_[0]);
                int ijk1 = static_cast<int>(double(input_->points[cp].y * inverse_leaf_size_[1]) - min_b_[1]);
                int ijk2 = static_cast<int>(double(input_->points[cp].z * inverse_leaf_size_[2]) - min_b_[2]);

                // Compute the centroid leaf index
                int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
                index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int>(idx), cp));
            }
        }
        // No distance filtering, process all data
        else {
            // First pass: go over all points and insert them into the index_vector vector
            // with calculated idx. Points with the same idx value will contribute to the
            // same point of resulting CloudPoint
            for (unsigned int cp = 0; cp < static_cast<unsigned int>(input_->points.size()); ++cp) {
                if (!input_->is_dense)
                // Check if the point is invalid
                    if (!pcl_isfinite(input_->points[cp].x) ||
                        !pcl_isfinite(input_->points[cp].y) ||
                        !pcl_isfinite(input_->points[cp].z))
                        continue;

                int ijk0 = static_cast<int>(double(input_->points[cp].x * inverse_leaf_size_[0]) - min_b_[0]);
                int ijk1 = static_cast<int>(double(input_->points[cp].y * inverse_leaf_size_[1]) - min_b_[1]);
                int ijk2 = static_cast<int>(double(input_->points[cp].z * inverse_leaf_size_[2]) - min_b_[2]);

                // Compute the centroid leaf index
                int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
                index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int>(idx), cp));
            }
        }

        // Second pass: sort the index_vector vector using value representing target cell as index
        // in effect all points belonging to the same output cell will be next to each other
        std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

        // Third pass: count output cells
        // we need to skip all the same, adjacenent idx values
        unsigned int total = 0;
        unsigned int index = 0;
        while (index < index_vector.size()) {
            unsigned int i = index + 1;
            while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
                ++i;
            ++total;
            index = i;
        }

        // Fourth pass: compute centroids, insert them into their final position
        output.points.resize(total);
        if (save_leaf_layout_) {
            try {
                // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
                uint32_t new_layout_size = div_b_[0] * div_b_[1] * div_b_[2];
                //This is the number of elements that need to be re-initialized to -1
                uint32_t reinit_size = std::min(static_cast<unsigned int>(new_layout_size), static_cast<unsigned int>(leaf_layout_.size()));
                for (uint32_t i = 0; i < reinit_size; i++) {
                    leaf_layout_[i] = -1;
                }
                leaf_layout_.resize(new_layout_size, -1);
            }
            catch (std::bad_alloc&) {
                throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                   "voxel_grid.hpp", "applyFilter");
            }
            catch (std::length_error&) {
                throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                   "voxel_grid.hpp", "applyFilter");
            }
        }

        index = 0;
        Eigen::VectorXd centroid = Eigen::VectorXd::Zero(centroid_size);
        Eigen::VectorXd temporary = Eigen::VectorXd::Zero(centroid_size);

        for (unsigned int cp = 0; cp < index_vector.size();) {
            // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
            if (!downsample_all_data_) {
                centroid[0] = input_->points[index_vector[cp].cloud_point_index].x;
                centroid[1] = input_->points[index_vector[cp].cloud_point_index].y;
                centroid[2] = input_->points[index_vector[cp].cloud_point_index].z;
            } else {
                // ---[ RGB special case
                if (rgba_index >= 0) {
                    // Fill r/g/b data, assuming that the order is BGRA
                    RGB rgb;
                    memcpy(&rgb, reinterpret_cast<const char*>(&input_->points[index_vector[cp].cloud_point_index]) + rgba_index, sizeof (RGB));
                    centroid[centroid_size - 3] = rgb.r;
                    centroid[centroid_size - 2] = rgb.g;
                    centroid[centroid_size - 1] = rgb.b;
                }
                for_each_type<FieldList>(NdCopyPointEigenFunctor<PointT>(input_->points[index_vector[cp].cloud_point_index], centroid));
            }

            unsigned int i = cp + 1;
            while (i < index_vector.size() && index_vector[i].idx == index_vector[cp].idx) {
                if (!downsample_all_data_) {
                    centroid[0] += input_->points[index_vector[i].cloud_point_index].x;
                    centroid[1] += input_->points[index_vector[i].cloud_point_index].y;
                    centroid[2] += input_->points[index_vector[i].cloud_point_index].z;
                } else {
                    // ---[ RGB special case
                    if (rgba_index >= 0) {
                        // Fill r/g/b data, assuming that the order is BGRA
                        RGB rgb;
                        memcpy(&rgb, reinterpret_cast<const char*>(&input_->points[index_vector[i].cloud_point_index]) + rgba_index, sizeof (RGB));
                        temporary[centroid_size - 3] = rgb.r;
                        temporary[centroid_size - 2] = rgb.g;
                        temporary[centroid_size - 1] = rgb.b;
                    }
                    for_each_type<FieldList>(NdCopyPointEigenFunctor<PointT>(input_->points[index_vector[i].cloud_point_index], temporary));
                    centroid += temporary;
                }
                ++i;
            }

            // index is centroid final position in resulting PointCloud
            if (save_leaf_layout_)
                leaf_layout_[index_vector[cp].idx] = index;

            centroid /= static_cast<double>(i - cp);

            // store centroid
            // Do we need to process all the fields?
            if (!downsample_all_data_) {
                output.points[index].x = centroid[0];
                output.points[index].y = centroid[1];
                output.points[index].z = centroid[2];
            } else {
                for_each_type<FieldList>(NdCopyEigenPointFunctor<PointT>(centroid, output.points[index]));
                // ---[ RGB special case
                if (rgba_index >= 0) {
                    // pack r/g/b into rgb
                    float r = centroid[centroid_size - 3], g = centroid[centroid_size - 2], b = centroid[centroid_size - 1];
                    int rgb = (static_cast<int>(r) << 16) | (static_cast<int>(g) << 8) | static_cast<int>(b);
                    memcpy(reinterpret_cast<char*>(&output.points[index]) + rgba_index, &rgb, sizeof (float));
                }
            }
            cp = i;
            ++index;
        }
        output.width = static_cast<uint32_t>(output.points.size());
    }
};


CLOUD_BLEND_DOUBLE_NAMESPACE_END