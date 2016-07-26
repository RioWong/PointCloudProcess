#pragma once
#include <boost/smart_ptr.hpp>
#include <flann/flann.hpp>
#include <vector>
#include "macros.h"
#include "point_cloud.h"
#include "point_traits.h"
#include "for_each_type.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
/** \brief @b PointRepresentation provides a set of methods for converting a point structs/object into an
  * n-dimensional vector.
  * \note This is an abstract class.  Subclasses must set nr_dimensions_ to the appropriate value in the constructor
  * and provide an implemention of the pure virtual copyToFloatArray method.
  * \author Michael Dixon
  */
template <typename PointT>
class PointRepresentation
{
protected:
    /** \brief The number of dimensions in this point's vector (i.e. the "k" in "k-D") */
    int nr_dimensions_;
    /** \brief A vector containing the rescale factor to apply to each dimension. */
    std::vector<double> alpha_;
    /** \brief Indicates whether this point representation is trivial. It is trivial if and only if the following
     *  conditions hold:
     *  - the relevant data consists only of float values
     *  - the vectorize operation directly copies the first nr_dimensions_ elements of PointT to the out array
     *  - sizeof(PointT) is a multiple of sizeof(float)
     *  In short, a trivial point representation converts the input point to a float array that is the same as if
     *  the point was reinterpret_casted to a float array of length nr_dimensions_ . This value says that this
     *  representation can be trivial; it is only trivial if setRescaleValues() has not been set.
     */
    bool trivial_;

public:
    typedef boost::shared_ptr<PointRepresentation<PointT> > Ptr;
    typedef boost::shared_ptr<const PointRepresentation<PointT> > ConstPtr;

    /** \brief Empty constructor */
    PointRepresentation() : nr_dimensions_(0), alpha_(0), trivial_(false)
    {
    }

    /** \brief Empty destructor */
    virtual ~PointRepresentation()
    {
    }

    /** \brief Copy point data from input point to a float array. This method must be overriden in all subclasses.
     *  \param[in] p The input point
     *  \param[out] out A pointer to a float array.
     */
    virtual void copyToDoubleArray(const PointT& p, double* out) const = 0;

    /** \brief Returns whether this point representation is trivial. It is trivial if and only if the following
     *  conditions hold:
     *  - the relevant data consists only of float values
     *  - the vectorize operation directly copies the first nr_dimensions_ elements of PointT to the out array
     *  - sizeof(PointT) is a multiple of sizeof(float)
     *  In short, a trivial point representation converts the input point to a float array that is the same as if
     *  the point was reinterpret_casted to a float array of length nr_dimensions_ . */
    inline bool isTrivial() const
    {
        return trivial_ && alpha_.empty();
    }

    /** \brief Verify that the input point is valid.
     *  \param p The point to validate
     */
    virtual bool
    isValid(const PointT& p) const
    {
        bool is_valid = true;

        if (trivial_) {
            const double* temp = reinterpret_cast<const double*>(&p);

            for (int i = 0; i < nr_dimensions_; ++i) {
                if (!pcl_isfinite(temp[i])) {
                    is_valid = false;
                    break;
                }
            }
        } else {
            double* temp = new double[nr_dimensions_];
            copyToDoubleArray(p, temp);

            for (int i = 0; i < nr_dimensions_; ++i) {
                if (!pcl_isfinite(temp[i])) {
                    is_valid = false;
                    break;
                }
            }
            delete [] temp;
        }
        return (is_valid);
    }

    /** \brief Convert input point into a vector representation, rescaling by \a alpha.
      * \param[in] p the input point
      * \param[out] out The output vector.  Can be of any type that implements the [] operator.
      */
    template <typename OutputType>
    void
    vectorize(const PointT& p, OutputType& out) const
    {
        double* temp = new double[nr_dimensions_];
        copyToDoubleArray(p, temp);
        if (alpha_.empty()) {
            for (int i = 0; i < nr_dimensions_; ++i)
                out[i] = temp[i];
        } else {
            for (int i = 0; i < nr_dimensions_; ++i)
                out[i] = temp[i] * alpha_[i];
        }
        delete [] temp;
    }

    /** \brief Set the rescale values to use when vectorizing points
      * \param[in] rescale_array The array/vector of rescale values.  Can be of any type that implements the [] operator.
      */
    void
    setRescaleValues(const double* rescale_array)
    {
        alpha_.resize(nr_dimensions_);
        for (int i = 0; i < nr_dimensions_; ++i)
            alpha_[i] = rescale_array[i];
    }

    /** \brief Return the number of dimensions in the point's vector representation. */
    inline int getNumberOfDimensions() const
    {
        return (nr_dimensions_);
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b DefaultPointRepresentation extends PointRepresentation to define default behavior for common point types.
 */
template <typename PointDefault>
class DefaultPointRepresentation : public PointRepresentation<PointDefault>
{
    using PointRepresentation<PointDefault>::nr_dimensions_;
    using PointRepresentation<PointDefault>::trivial_;

public:
    // Boost shared pointers
    typedef boost::shared_ptr<DefaultPointRepresentation<PointDefault> > Ptr;
    typedef boost::shared_ptr<const DefaultPointRepresentation<PointDefault> > ConstPtr;

    DefaultPointRepresentation()
    {
        // If point type is unknown, assume it's a struct/array of floats, and compute the number of dimensions
        nr_dimensions_ = sizeof (PointDefault) / sizeof (double);
        // Limit the default representation to the first 3 elements
        if (nr_dimensions_ > 3) nr_dimensions_ = 3;

        trivial_ = true;
    }

    virtual ~DefaultPointRepresentation()
    {
    }

    inline Ptr
    makeShared() const
    {
        return (Ptr(new DefaultPointRepresentation<PointDefault>(*this)));
    }

    virtual void
    copyToDoubleArray(const PointDefault& p, double* out) const
    {
        // If point type is unknown, treat it as a struct/array of floats
        const double* ptr = reinterpret_cast<const double*>(&p);
        for (int i = 0; i < nr_dimensions_; ++i)
            out[i] = ptr[i];
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b DefaulFeatureRepresentation extends PointRepresentation and is intended to be used when defining the
  * default behavior for feature descriptor types (i.e., copy each element of each field into a float array).
  */
template <typename PointDefault>
class DefaultFeatureRepresentation : public PointRepresentation<PointDefault>
{
protected:
    using PointRepresentation<PointDefault>::nr_dimensions_;

private:
    struct IncrementFunctor
    {
        IncrementFunctor(int& n) : n_(n)
        {
            n_ = 0;
        }

        template <typename Key>
        inline void operator ()()
        {
            n_ += traits::datatype<PointDefault, Key>::size;
        }

    private:
        int& n_;
    };

    struct NdCopyPointFunctor
    {
        typedef typename traits::POD<PointDefault>::type Pod;

        NdCopyPointFunctor(const PointDefault& p1, double* p2)
            : p1_(reinterpret_cast<const Pod&>(p1)), p2_(p2), f_idx_(0)
        {
        }

        template <typename Key>
        inline void operator()()
        {
            typedef typename traits::datatype<PointDefault, Key>::type FieldT;
            const int NrDims = traits::datatype<PointDefault, Key>::size;
            Helper<Key, FieldT, NrDims>::copyPoint(p1_, p2_, f_idx_);
        }

        // Copy helper for scalar fields
        template <typename Key, typename FieldT, int NrDims>
        struct Helper
        {
            static void copyPoint(const Pod& p1, double* p2, int& f_idx)
            {
                const uint8_t* data_ptr = reinterpret_cast<const uint8_t *>(&p1) +
                    traits::offset<PointDefault, Key>::value;
                p2[f_idx++] = *reinterpret_cast<const FieldT*>(data_ptr);
            }
        };

        // Copy helper for array fields
        template <typename Key, typename FieldT, int NrDims>
        struct Helper<Key, FieldT[NrDims], NrDims>
        {
            static void copyPoint(const Pod& p1, double* p2, int& f_idx)
            {
                const uint8_t* data_ptr = reinterpret_cast<const uint8_t *>(&p1) +
                    traits::offset<PointDefault, Key>::value;
                int nr_dims = NrDims;
                const FieldT* array = reinterpret_cast<const FieldT *>(data_ptr);
                for (int i = 0; i < nr_dims; ++i) {
                    p2[f_idx++] = array[i];
                }
            }
        };

    private:
        const Pod& p1_;
        double* p2_;
        int f_idx_;
    };

public:
    // Boost shared pointers
    typedef typename boost::shared_ptr<DefaultFeatureRepresentation<PointDefault> > Ptr;
    typedef typename boost::shared_ptr<const DefaultFeatureRepresentation<PointDefault> > ConstPtr;
    typedef typename traits::fieldList<PointDefault>::type FieldList;

    DefaultFeatureRepresentation()
    {
        nr_dimensions_ = 0; // zero-out the nr_dimensions_ before it gets incremented
        for_each_type<FieldList>(IncrementFunctor(nr_dimensions_));
    }

    inline Ptr
    makeShared() const
    {
        return (Ptr(new DefaultFeatureRepresentation<PointDefault>(*this)));
    }

    virtual void
    copyToDoubleArray(const PointDefault& p, double* out) const
    {
        for_each_type<FieldList>(NdCopyPointFunctor(p, out));
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b CustomPointRepresentation extends PointRepresentation to allow for sub-part selection on the point.
 */
template <typename PointDefault>
class CustomPointRepresentation : public PointRepresentation<PointDefault>
{
    using PointRepresentation<PointDefault>::nr_dimensions_;

public:
    // Boost shared pointers
    typedef boost::shared_ptr<CustomPointRepresentation<PointDefault> > Ptr;
    typedef boost::shared_ptr<const CustomPointRepresentation<PointDefault> > ConstPtr;

    /** \brief Constructor
      * \param[in] max_dim the maximum number of dimensions to use
      * \param[in] start_dim the starting dimension
      */
    CustomPointRepresentation(const int max_dim = 3, const int start_dim = 0)
        : max_dim_(max_dim), start_dim_(start_dim)
    {
        // If point type is unknown, assume it's a struct/array of floats, and compute the number of dimensions
        nr_dimensions_ = static_cast<int>(sizeof (PointDefault) / sizeof (double)) - start_dim_;
        // Limit the default representation to the first 3 elements
        if (nr_dimensions_ > max_dim_)
            nr_dimensions_ = max_dim_;
    }

    inline Ptr
    makeShared() const
    {
        return Ptr(new CustomPointRepresentation<PointDefault>(*this));
    }

    /** \brief Copy the point data into a float array
      * \param[in] p the input point
      * \param[out] out the resultant output array
      */
    virtual void
    copyToDoubleArray(const PointDefault& p, double* out) const
    {
        // If point type is unknown, treat it as a struct/array of floats
        const double* ptr = (reinterpret_cast<const double*>(&p)) + start_dim_;
        for (int i = 0; i < nr_dimensions_; ++i)
            out[i] = ptr[i];
    }

protected:
    /** \brief Use at most this many dimensions (i.e. the "k" in "k-D" is at most max_dim_) -- \note float fields are assumed */
    int max_dim_;
    /** \brief Use dimensions only starting with this one (i.e. the "k" in "k-D" is = dim - start_dim_) -- \note float fields are assumed */
    int start_dim_;
};

/** \brief KdTree represents the base spatial locator class for kd-tree implementations.
  * \author Radu B Rusu, Bastian Steder, Michael Dixon
  * \ingroup kdtree
  */
template <typename PointT>
class KdTree
{
public:
    typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
    typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

    typedef CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloud<PointT> PointCloud;
    typedef boost::shared_ptr<PointCloud> PointCloudPtr;
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    typedef CLOUD_BLEND_DOUBLE_NAMESPACE::PointRepresentation<PointT> PointRepresentation;
    //typedef boost::shared_ptr<PointRepresentation> PointRepresentationPtr;
    typedef boost::shared_ptr<const PointRepresentation> PointRepresentationConstPtr;

    // Boost shared pointers
    typedef boost::shared_ptr<KdTree<PointT> > Ptr;
    typedef boost::shared_ptr<const KdTree<PointT> > ConstPtr;

    /** \brief Empty constructor for KdTree. Sets some internal values to their defaults. 
      * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise. 
      */
    KdTree(bool sorted = true) : input_(), indices_(),
                                 epsilon_(0.0f), min_pts_(1), sorted_(sorted),
                                 point_representation_(new DefaultPointRepresentation < PointT > )
    {
    };

    /** \brief Provide a pointer to the input dataset.
      * \param[in] cloud the const boost shared pointer to a PointCloud message
      * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
      */
    virtual void
    setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr())
    {
        input_ = cloud;
        indices_ = indices;
    }

    /** \brief Get a pointer to the vector of indices used. */
    inline IndicesConstPtr
    getIndices() const
    {
        return (indices_);
    }

    /** \brief Get a pointer to the input point cloud dataset. */
    inline PointCloudConstPtr
    getInputCloud() const
    {
        return (input_);
    }

    /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors. 
      * \param[in] point_representation the const boost shared pointer to a PointRepresentation
      */
    inline void
    setPointRepresentation(const PointRepresentationConstPtr& point_representation)
    {
        point_representation_ = point_representation;
        setInputCloud(input_, indices_); // Makes sense in derived classes to reinitialize the tree
    }

    /** \brief Get a pointer to the point representation used when converting points into k-D vectors. */
    inline PointRepresentationConstPtr
    getPointRepresentation() const
    {
        return (point_representation_);
    }

    /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
    virtual ~KdTree()
    {
    };

    /** \brief Search for k-nearest neighbors for the given query point.
      * \param[in] p_q the given query point
      * \param[in] k the number of neighbors to search for
      * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k 
      * a priori!)
      * \return number of neighbors found
      */
    virtual int
    nearestKSearch(const PointT& p_q, int k,
                   std::vector<int>& k_indices, std::vector<double>& k_sqr_distances) const = 0;

    /** \brief Search for k-nearest neighbors for the given query point.
      * 
      * \attention This method does not do any bounds checking for the input index
      * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
      * 
      * \param[in] cloud the point cloud data
      * \param[in] index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
      * \param[in] k the number of neighbors to search for
      * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k 
      * a priori!)
      * 
      * \return number of neighbors found
      * 
      * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
      */
    virtual int
    nearestKSearch(const PointCloud& cloud, int index, int k,
                   std::vector<int>& k_indices, std::vector<double>& k_sqr_distances) const
    {
        assert (index >= 0 && index < static_cast<int> (cloud.points.size ()) && "Out-of-bounds error in nearestKSearch!");
        return (nearestKSearch(cloud.points[index], k, k_indices, k_sqr_distances));
    }

    /** \brief Search for k-nearest neighbors for the given query point. 
      * This method accepts a different template parameter for the point type.
      * \param[in] point the given query point
      * \param[in] k the number of neighbors to search for
      * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k 
      * a priori!)
      * \return number of neighbors found
      */
    template <typename PointTDiff>
    inline int
    nearestKSearchT(const PointTDiff& point, int k,
                    std::vector<int>& k_indices, std::vector<double>& k_sqr_distances) const
    {
        PointT p;
        // Copy all the data fields from the input cloud to the output one
        typedef typename traits::fieldList<PointT>::type FieldListInT;
        typedef typename traits::fieldList<PointTDiff>::type FieldListOutT;
        typedef typename intersect<FieldListInT, FieldListOutT>::type FieldList;
        for_each_type<FieldList>(NdConcatenateFunctor<PointTDiff, PointT>(point, p));
        return (nearestKSearch(p, k, k_indices, k_sqr_distances));
    }

    /** \brief Search for k-nearest neighbors for the given query point (zero-copy).
      * 
      * \attention This method does not do any bounds checking for the input index
      * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
      * 
      * \param[in] index a \a valid index representing a \a valid query point in the dataset given 
      * by \a setInputCloud. If indices were given in setInputCloud, index will be the position in 
      * the indices vector.
      * 
      * \param[in] k the number of neighbors to search for
      * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k 
      * a priori!)
      * \return number of neighbors found
      * 
      * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
      */
    virtual int
    nearestKSearch(int index, int k,
                   std::vector<int>& k_indices, std::vector<double>& k_sqr_distances) const
    {
        if (indices_ == NULL) {
            assert (index >= 0 && index < static_cast<int> (input_->points.size ()) && "Out-of-bounds error in nearestKSearch!");
            return (nearestKSearch(input_->points[index], k, k_indices, k_sqr_distances));
        } else {
            assert (index >= 0 && index < static_cast<int> (indices_->size ()) && "Out-of-bounds error in nearestKSearch!");
            return (nearestKSearch(input_->points[(*indices_)[index]], k, k_indices, k_sqr_distances));
        }
    }

    /** \brief Search for all the nearest neighbors of the query point in a given radius.
      * \param[in] p_q the given query point
      * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
      * \param[out] k_indices the resultant indices of the neighboring points
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
      * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
      * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
      * returned.
      * \return number of neighbors found in radius
      */
    virtual int
    radiusSearch(const PointT& p_q, double radius, std::vector<int>& k_indices,
                 std::vector<double>& k_sqr_distances, unsigned int max_nn = 0) const = 0;

    /** \brief Search for all the nearest neighbors of the query point in a given radius.
      * 
      * \attention This method does not do any bounds checking for the input index
      * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
      * 
      * \param[in] cloud the point cloud data
      * \param[in] index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
      * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
      * \param[out] k_indices the resultant indices of the neighboring points
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
      * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
      * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
      * returned.
      * \return number of neighbors found in radius
      * 
      * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
      */
    virtual int
    radiusSearch(const PointCloud& cloud, int index, double radius,
                 std::vector<int>& k_indices, std::vector<double>& k_sqr_distances,
                 unsigned int max_nn = 0) const
    {
        assert (index >= 0 && index < static_cast<int> (cloud.points.size ()) && "Out-of-bounds error in radiusSearch!");
        return (radiusSearch(cloud.points[index], radius, k_indices, k_sqr_distances, max_nn));
    }

    /** \brief Search for all the nearest neighbors of the query point in a given radius.
      * \param[in] point the given query point
      * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
      * \param[out] k_indices the resultant indices of the neighboring points
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
      * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
      * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
      * returned.
      * \return number of neighbors found in radius
      */
    template <typename PointTDiff>
    inline int
    radiusSearchT(const PointTDiff& point, double radius, std::vector<int>& k_indices,
                  std::vector<double>& k_sqr_distances, unsigned int max_nn = 0) const
    {
        PointT p;
        // Copy all the data fields from the input cloud to the output one
        typedef typename traits::fieldList<PointT>::type FieldListInT;
        typedef typename traits::fieldList<PointTDiff>::type FieldListOutT;
        typedef typename intersect<FieldListInT, FieldListOutT>::type FieldList;
        for_each_type<FieldList>(NdConcatenateFunctor<PointTDiff, PointT>(point, p));
        return (radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn));
    }

    /** \brief Search for all the nearest neighbors of the query point in a given radius (zero-copy).
      * 
      * \attention This method does not do any bounds checking for the input index
      * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
      * 
      * \param[in] index a \a valid index representing a \a valid query point in the dataset given 
      * by \a setInputCloud. If indices were given in setInputCloud, index will be the position in 
      * the indices vector.
      * 
      * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
      * \param[out] k_indices the resultant indices of the neighboring points
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
      * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
      * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
      * returned.
      * \return number of neighbors found in radius
      * 
      * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
      */
    virtual int
    radiusSearch(int index, double radius, std::vector<int>& k_indices,
                 std::vector<double>& k_sqr_distances, unsigned int max_nn = 0) const
    {
        if (indices_ == NULL) {
            assert (index >= 0 && index < static_cast<int> (input_->points.size ()) && "Out-of-bounds error in radiusSearch!");
            return (radiusSearch(input_->points[index], radius, k_indices, k_sqr_distances, max_nn));
        } else {
            assert (index >= 0 && index < static_cast<int> (indices_->size ()) && "Out-of-bounds error in radiusSearch!");
            return (radiusSearch(input_->points[(*indices_)[index]], radius, k_indices, k_sqr_distances, max_nn));
        }
    }

    /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
      * \param[in] eps precision (error bound) for nearest neighbors searches
      */
    virtual inline void
    setEpsilon(float eps)
    {
        epsilon_ = eps;
    }

    /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
    inline float
    getEpsilon() const
    {
        return (epsilon_);
    }

    /** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain. 
      * \param[in] min_pts the minimum number of neighbors in a viable neighborhood 
      */
    inline void
    setMinPts(int min_pts)
    {
        min_pts_ = min_pts;
    }

    /** \brief Get the minimum allowed number of k nearest neighbors points that a viable result must contain. */
    inline int
    getMinPts() const
    {
        return (min_pts_);
    }

protected:
    /** \brief The input point cloud dataset containing the points we need to use. */
    PointCloudConstPtr input_;

    /** \brief A pointer to the vector of point indices to use. */
    IndicesConstPtr indices_;

    /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
    float epsilon_;

    /** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain. */
    int min_pts_;

    /** \brief Return the radius search neighbours sorted **/
    bool sorted_;

    /** \brief For converting different point structures into k-dimensional vectors for nearest-neighbor search. */
    PointRepresentationConstPtr point_representation_;

    /** \brief Class getName method. */
    virtual std::string
    getName() const = 0;
};

/** \brief KdTreeFLANN is a generic type of 3D spatial locator using kD-tree structures. The class is making use of
  * the FLANN (Fast Library for Approximate Nearest Neighbor) project by Marius Muja and David Lowe.
  *
  * \author Radu B. Rusu, Marius Muja
  * \ingroup kdtree 
  */
template <typename PointT, typename Dist = flann::L2_Simple<double> >
class KdTreeFLANN : public KdTree<PointT>
{
public:
    using KdTree<PointT>::input_;
    using KdTree<PointT>::indices_;
    using KdTree<PointT>::epsilon_;
    using KdTree<PointT>::sorted_;
    using KdTree<PointT>::point_representation_;
    using KdTree<PointT>::nearestKSearch;
    using KdTree<PointT>::radiusSearch;

    typedef typename KdTree<PointT>::PointCloud PointCloud;
    typedef typename KdTree<PointT>::PointCloudConstPtr PointCloudConstPtr;

    typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
    typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

    typedef flann::Index<Dist> FLANNIndex;

    // Boost shared pointers
    typedef boost::shared_ptr<KdTreeFLANN<PointT> > Ptr;
    typedef boost::shared_ptr<const KdTreeFLANN<PointT> > ConstPtr;

    /** \brief Default Constructor for KdTreeFLANN.
      * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise. 
      *
      * By setting sorted to false, the \ref radiusSearch operations will be faster.
      */
    KdTreeFLANN(bool sorted = true) :
        KdTree<PointT>(sorted),
        flann_index_(NULL), cloud_(NULL),
        index_mapping_(), identity_mapping_(false),
        dim_(0), total_nr_points_(0),
        param_k_(flann::SearchParams(-1, epsilon_)),
        param_radius_(flann::SearchParams(-1, epsilon_, sorted))
    {
    }

    /** \brief Copy constructor
      * \param[in] tree the tree to copy into this
      */
    KdTreeFLANN(const KdTreeFLANN<PointT>& k) :
        KdTree<PointT>(false),
        flann_index_(NULL), cloud_(NULL),
        index_mapping_(), identity_mapping_(false),
        dim_(0), total_nr_points_(0),
        param_k_(flann::SearchParams(-1, epsilon_)),
        param_radius_(flann::SearchParams(-1, epsilon_, false))
    {
        *this = k;
    }

    /** \brief Copy operator
      * \param[in] tree the tree to copy into this
      */
    inline KdTreeFLANN<PointT>&
    operator =(const KdTreeFLANN<PointT>& k)
    {
        KdTree<PointT>::operator=(k);
        flann_index_ = k.flann_index_;
        cloud_ = k.cloud_;
        index_mapping_ = k.index_mapping_;
        identity_mapping_ = k.identity_mapping_;
        dim_ = k.dim_;
        total_nr_points_ = k.total_nr_points_;
        param_k_ = k.param_k_;
        param_radius_ = k.param_radius_;
        return (*this);
    }

    /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
      * \param[in] eps precision (error bound) for nearest neighbors searches
      */
    inline void
    setEpsilon(float eps)
    {
        epsilon_ = eps;
        param_k_ = flann::SearchParams(-1, epsilon_);
        param_radius_ = flann::SearchParams(-1, epsilon_, sorted_);
    }

    inline void
    setSortedResults(bool sorted)
    {
        sorted_ = sorted;
        param_k_ = flann::SearchParams(-1, epsilon_);
        param_radius_ = flann::SearchParams(-1, epsilon_, sorted_);
    }

    inline Ptr makeShared()
    {
        return Ptr(new KdTreeFLANN<PointT>(*this));
    }

    /** \brief Destructor for KdTreeFLANN. 
      * Deletes all allocated data arrays and destroys the kd-tree structures. 
      */
    virtual ~KdTreeFLANN()
    {
        cleanup();
    }

    /** \brief Provide a pointer to the input dataset.
      * \param[in] cloud the const boost shared pointer to a PointCloud message
      * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
      */
    void
    setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr())
    {
        cleanup(); // Perform an automatic cleanup of structures

        epsilon_ = 0.0f; // default error bound value
        dim_ = point_representation_->getNumberOfDimensions(); // Number of dimensions - default is 3 = xyz

        input_ = cloud;
        indices_ = indices;

        // Allocate enough data
        if (!input_) {
            //PCL_ERROR("[pcl::KdTreeFLANN::setInputCloud] Invalid input!\n");
            return;
        }
        if (indices != NULL) {
            convertCloudToArray(*input_, *indices_);
        } else {
            convertCloudToArray(*input_);
        }
        total_nr_points_ = static_cast<int>(index_mapping_.size());

        flann_index_ = new FLANNIndex(flann::Matrix<double>(cloud_, index_mapping_.size(), dim_),
                                      flann::KDTreeSingleIndexParams(15)); // max 15 points/leaf
        flann_index_->buildIndex();
    }

    /** \brief Search for k-nearest neighbors for the given query point.
      * 
      * \attention This method does not do any bounds checking for the input index
      * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
      * 
      * \param[in] point a given \a valid (i.e., finite) query point
      * \param[in] k the number of neighbors to search for
      * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k 
      * a priori!)
      * \return number of neighbors found
      * 
      * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
      */
    int
    nearestKSearch(const PointT& point, int k,
                   std::vector<int>& k_indices, std::vector<double>& k_sqr_distances) const
    {
        assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

        if (k > total_nr_points_)
            k = total_nr_points_;

        k_indices.resize(k);
        k_sqr_distances.resize(k);

        std::vector<double> query(dim_);
        point_representation_->vectorize(static_cast<PointT>(point), query);

        flann::Matrix<int> k_indices_mat(&k_indices[0], 1, k);
        flann::Matrix<double> k_distances_mat(&k_sqr_distances[0], 1, k);
        // Wrap the k_indices and k_sqr_distances vectors (no data copy)
        flann_index_->knnSearch(flann::Matrix<double>(&query[0], 1, dim_),
                                k_indices_mat, k_distances_mat,
                                k, param_k_);

        // Do mapping to original point cloud
        if (!identity_mapping_) {
            for (size_t i = 0; i < static_cast<size_t>(k); ++i) {
                int& neighbor_index = k_indices[i];
                neighbor_index = index_mapping_[neighbor_index];
            }
        }

        return (k);
    }

    /** \brief Search for all the nearest neighbors of the query point in a given radius.
      * 
      * \attention This method does not do any bounds checking for the input index
      * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
      * 
      * \param[in] point a given \a valid (i.e., finite) query point
      * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
      * \param[out] k_indices the resultant indices of the neighboring points
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
      * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
      * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
      * returned.
      * \return number of neighbors found in radius
      *
      * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
      */
    int
    radiusSearch(const PointT& point, double radius, std::vector<int>& k_indices,
                 std::vector<double>& k_sqr_distances, unsigned int max_nn = 0) const
    {
        assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");

        std::vector<double> query(dim_);
        point_representation_->vectorize(static_cast<PointT>(point), query);

        // Has max_nn been set properly?
        if (max_nn == 0 || max_nn > static_cast<unsigned int>(total_nr_points_))
            max_nn = total_nr_points_;

        std::vector<std::vector<int> > indices(1);
        std::vector<std::vector<double> > dists(1);

        flann::SearchParams params(param_radius_);
        if (max_nn == static_cast<unsigned int>(total_nr_points_))
            params.max_neighbors = -1; // return all neighbors in radius
        else
            params.max_neighbors = max_nn;

        int neighbors_in_radius = flann_index_->radiusSearch(flann::Matrix<double>(&query[0], 1, dim_),
                                                             indices,
                                                             dists,
                                                             static_cast<double>(radius * radius),
                                                             params);

        k_indices = indices[0];
        k_sqr_distances = dists[0];

        // Do mapping to original point cloud
        if (!identity_mapping_) {
            for (int i = 0; i < neighbors_in_radius; ++i) {
                int& neighbor_index = k_indices[i];
                neighbor_index = index_mapping_[neighbor_index];
            }
        }

        return (neighbors_in_radius);
    }

private:
    /** \brief Internal cleanup method. */
    void
    cleanup()
    {
        if (flann_index_)
            delete flann_index_;

        // Data array cleanup
        if (cloud_) {
            free(cloud_);
            cloud_ = NULL;
        }
        index_mapping_.clear();

        if (indices_)
            indices_.reset();
    }

    /** \brief Converts a PointCloud to the internal FLANN point array representation. Returns the number
      * of points.
      * \param cloud the PointCloud 
      */
    void
    convertCloudToArray(const PointCloud& cloud)
    {
        // No point in doing anything if the array is empty
        if (cloud.points.empty()) {
            cloud_ = NULL;
            return;
        }

        int original_no_of_points = static_cast<int>(cloud.points.size());

        cloud_ = static_cast<double*>(malloc(original_no_of_points * dim_ * sizeof (double)));
        double* cloud_ptr = cloud_;
        index_mapping_.reserve(original_no_of_points);
        identity_mapping_ = true;

        for (int cloud_index = 0; cloud_index < original_no_of_points; ++cloud_index) {
            // Check if the point is invalid
            if (!point_representation_->isValid(cloud.points[cloud_index])) {
                identity_mapping_ = false;
                continue;
            }

            index_mapping_.push_back(cloud_index);

            point_representation_->vectorize(cloud.points[cloud_index], cloud_ptr);
            cloud_ptr += dim_;
        }
    }

    /** \brief Converts a PointCloud with a given set of indices to the internal FLANN point array
      * representation. Returns the number of points.
      * \param[in] cloud the PointCloud data
      * \param[in] indices the point cloud indices
     */
    void
    convertCloudToArray(const PointCloud& cloud, const std::vector<int>& indices)
    {
        // No point in doing anything if the array is empty
        if (cloud.points.empty()) {
            cloud_ = NULL;
            return;
        }

        int original_no_of_points = static_cast<int>(indices.size());

        cloud_ = static_cast<double*>(malloc(original_no_of_points * dim_ * sizeof (double)));
        double* cloud_ptr = cloud_;
        index_mapping_.reserve(original_no_of_points);
        // its a subcloud -> false
        // true only identity: 
        //     - indices size equals cloud size
        //     - indices only contain values between 0 and cloud.size - 1
        //     - no index is multiple times in the list
        //     => index is complete
        // But we can not guarantee that => identity_mapping_ = false
        identity_mapping_ = false;

        for (std::vector<int>::const_iterator iIt = indices.begin(); iIt != indices.end(); ++iIt) {
            // Check if the point is invalid
            if (!point_representation_->isValid(cloud.points[*iIt]))
                continue;

            // map from 0 - N -> indices [0] - indices [N]
            index_mapping_.push_back(*iIt); // If the returned index should be for the indices vector

            point_representation_->vectorize(cloud.points[*iIt], cloud_ptr);
            cloud_ptr += dim_;
        }
    }

private:
    /** \brief Class getName method. */
    virtual std::string
    getName() const
    {
        return ("KdTreeFLANN");
    }

    /** \brief A FLANN index object. */
    FLANNIndex* flann_index_;

    /** \brief Internal pointer to data. */
    double* cloud_;

    /** \brief mapping between internal and external indices. */
    std::vector<int> index_mapping_;

    /** \brief whether the mapping bwwteen internal and external indices is identity */
    bool identity_mapping_;

    /** \brief Tree dimensionality (i.e. the number of dimensions per point). */
    int dim_;

    /** \brief The total size of the data (either equal to the number of points in the input cloud or to the number of indices - if passed). */
    int total_nr_points_;

    /** \brief The KdTree search parameters for K-nearest neighbors. */
    flann::SearchParams param_k_;

    /** \brief The KdTree search parameters for radius search. */
    flann::SearchParams param_radius_;
};

CLOUD_BLEND_DOUBLE_NAMESPACE_END