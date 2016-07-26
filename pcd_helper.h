#pragma once
#include <boost/algorithm/string.hpp>
#include "file_io_helper.h" 
#include "sensor_point_field.h"
#include "sensor_point_cloud2.h"
#include "point_cloud.h"
#include "point_type.h"
#include "channel_properties.h"
#include "sensor_image.h"
#include "lzf.h"
#include "conversions.h"
#include "exception.h"
//#include "point_traits.h"

#ifdef _WIN32
# include <io.h>
# include <windows.h>
# define pcl_open                    _open
# define pcl_close(fd)               _close(fd)
# define pcl_lseek(fd,offset,origin) _lseek(fd,offset,origin)
#else
# include <sys/mman.h>
#include <fcntl.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)
#endif

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
template <typename PointT>
void
getFields(const PointCloud<PointT>&, std::vector<sensor_msgs::PointField>& fields)
{
    fields.clear();
    // Get the fields list
    for_each_type<typename traits::fieldList<PointT>::type>(detail::FieldAdder<PointT>(fields));
}

class PCDReader : public FileReader
{
public:
    /** Empty constructor */
    PCDReader() : FileReader()
    {
    }

    /** Empty destructor */
    ~PCDReader()
    {
    }

    /** \brief Various PCD file versions.
      *
      * PCD_V6 represents PCD files with version 0.6, which contain the following fields:
      *   - lines beginning with # are treated as comments
      *   - FIELDS ...
      *   - SIZE ...
      *   - TYPE ...
      *   - COUNT ...
      *   - WIDTH ...
      *   - HEIGHT ...
      *   - POINTS ...
      *   - DATA ascii/binary
      * 
      * Everything that follows \b DATA is intepreted as data points and
      * will be read accordingly.
      *
      * PCD_V7 represents PCD files with version 0.7 and has an important
      * addon: it adds sensor origin/orientation (aka viewpoint) information
      * to a dataset through the use of a new header field:
      *   - VIEWPOINT tx ty tz qw qx qy qz
      */
    enum
    {
        PCD_V6 = 0,
        PCD_V7 = 1
    };

    /** \brief Read a point cloud data header from a PCD file. 
      *
      * Load only the meta information (number of points, their types, etc),
      * and not the points themselves, from a given PCD file. Useful for fast
      * evaluation of the underlying data structure.
      *
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
      * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
      * \param[out] orientation the sensor acquisition orientation (only for > PCD_V7 - identity if not present)
      * \param[out] pcd_version the PCD version of the file (i.e., PCD_V6, PCD_V7)
      * \param[out] data_type the type of data (0 = ASCII, 1 = Binary, 2 = Binary compressed) 
      * \param[out] data_idx the offset of cloud data within the file
      * \param[in] offset the offset of where to expect the PCD Header in the
      * file (optional parameter). One usage example for setting the offset
      * parameter is for reading data from a TAR "archive containing multiple
      * PCD files: TAR files always add a 512 byte header in front of the
      * actual file, so set the offset to the next byte after the header
      * (e.g., 513).
      *
      * \return
      *  * < 0 (-1) on error
      *  * == 0 on success
      */
    int
    readHeader(const std::string& file_name, sensor_msgs::PointCloud2& cloud,
               Eigen::Vector4d& origin, Eigen::Quaterniond& orientation, int& pcd_version,
               int& data_type, unsigned int& data_idx, const int offset = 0);


    /** \brief Read a point cloud data header from a PCD file. 
      *
      * Load only the meta information (number of points, their types, etc),
      * and not the points themselves, from a given PCD file. Useful for fast
      * evaluation of the underlying data structure.
      *
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
      * \param[in] offset the offset of where to expect the PCD Header in the
      * file (optional parameter). One usage example for setting the offset
      * parameter is for reading data from a TAR "archive containing multiple
      * PCD files: TAR files always add a 512 byte header in front of the
      * actual file, so set the offset to the next byte after the header
      * (e.g., 513).
      *
      * \return
      *  * < 0 (-1) on error
      *  * == 0 on success
      */
    int
    readHeader(const std::string& file_name, sensor_msgs::PointCloud2& cloud, const int offset = 0);

    /** \brief Read a point cloud data header from a PCD file. 
      *
      * Load only the meta information (number of points, their types, etc),
      * and not the points themselves, from a given PCD file. Useful for fast
      * evaluation of the underlying data structure.
      *
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant point cloud dataset (only the properties will be filled)
      * \param[out] pcd_version the PCD version of the file (either PCD_V6 or PCD_V7)
      * \param[out] data_type the type of data (0 = ASCII, 1 = Binary, 2 = Binary compressed) 
      * \param[out] data_idx the offset of cloud data within the file
      * \param[in] offset the offset of where to expect the PCD Header in the
      * file (optional parameter). One usage example for setting the offset
      * parameter is for reading data from a TAR "archive containing multiple
      * PCD files: TAR files always add a 512 byte header in front of the
      * actual file, so set the offset to the next byte after the header
      * (e.g., 513).
      *
      * \return
      *  * < 0 (-1) on error
      *  * == 0 on success
      *
      */
    //       int 
    //       readHeaderEigen (const std::string &file_name, PointCloud<Eigen::MatrixXf> &cloud,
    //                        int &pcd_version, int &data_type, unsigned int &data_idx, const int offset = 0);

    /** \brief Read a point cloud data from a PCD file and store it into a sensor_msgs/PointCloud2.
      * \param[in] file_name the name of the file containing the actual PointCloud data
      * \param[out] cloud the resultant PointCloud message read from disk
      * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
      * \param[out] orientation the sensor acquisition orientation (only for > PCD_V7 - identity if not present)
      * \param[out] pcd_version the PCD version of the file (either PCD_V6 or PCD_V7)
      * \param[in] offset the offset of where to expect the PCD Header in the
      * file (optional parameter). One usage example for setting the offset
      * parameter is for reading data from a TAR "archive containing multiple
      * PCD files: TAR files always add a 512 byte header in front of the
      * actual file, so set the offset to the next byte after the header
      * (e.g., 513).
      *
      * \return
      *  * < 0 (-1) on error
      *  * == 0 on success
      */
    int
    read(const std::string& file_name, sensor_msgs::PointCloud2& cloud,
         Eigen::Vector4d& origin, Eigen::Quaterniond& orientation, int& pcd_version, const int offset = 0);

    /** \brief Read a point cloud data from a PCD (PCD_V6) and store it into a sensor_msgs/PointCloud2.
      * 
      * \note This function is provided for backwards compatibility only and
      * it can only read PCD_V6 files correctly, as sensor_msgs::PointCloud2
      * does not contain a sensor origin/orientation. Reading any file 
      * > PCD_V6 will generate a warning. 
      *
      * \param[in] file_name the name of the file containing the actual PointCloud data
      * \param[out] cloud the resultant PointCloud message read from disk
      * \param[in] offset the offset of where to expect the PCD Header in the
      * file (optional parameter). One usage example for setting the offset
      * parameter is for reading data from a TAR "archive containing multiple
      * PCD files: TAR files always add a 512 byte header in front of the
      * actual file, so set the offset to the next byte after the header
      * (e.g., 513).
      *
      * \return
      *  * < 0 (-1) on error
      *  * == 0 on success
      */
    int
    read(const std::string& file_name, sensor_msgs::PointCloud2& cloud, const int offset = 0);

    /** \brief Read a point cloud data from any PCD file, and convert it to the given template format.
      * \param[in] file_name the name of the file containing the actual PointCloud data
      * \param[out] cloud the resultant PointCloud message read from disk
      * \param[in] offset the offset of where to expect the PCD Header in the
      * file (optional parameter). One usage example for setting the offset
      * parameter is for reading data from a TAR "archive containing multiple
      * PCD files: TAR files always add a 512 byte header in front of the
      * actual file, so set the offset to the next byte after the header
      * (e.g., 513).
      *
      * \return
      *  * < 0 (-1) on error
      *  * == 0 on success
      */
    template <typename PointT>
    int
    read(const std::string& file_name, PointCloud<PointT>& cloud, const int offset = 0)
    {
        sensor_msgs::PointCloud2 blob;
        int pcd_version;
        int res = read(file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_,
                       pcd_version, offset);

        // If no error, convert the data
        if (res == 0)
            fromROSMsg(blob, cloud);
        return (res);
    }

    /** \brief Read a point cloud data from any PCD file, and convert it to a PointCloud<Eigen::MatrixXf> format.
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] file_name the name of the file containing the actual PointCloud data
      * \param[out] cloud the resultant PointCloud message read from disk
      * \param[in] offset the offset of where to expect the PCD Header in the
      * file (optional parameter). One usage example for setting the offset
      * parameter is for reading data from a TAR "archive containing multiple
      * PCD files: TAR files always add a 512 byte header in front of the
      * actual file, so set the offset to the next byte after the header
      * (e.g., 513).
      *
      * \return
      *  * < 0 (-1) on error
      *  * == 0 on success
      */
    //       int
    //       readEigen (const std::string &file_name, PointCloud<Eigen::MatrixXf> &cloud, const int offset = 0);
};

class PCDWriter : public FileWriter
{
public:
    PCDWriter() : FileWriter(), map_synchronization_(false)
    {
    }

    ~PCDWriter()
    {
    }

    /** \brief Set whether mmap() synchornization via msync() is desired before munmap() calls. 
      * Setting this to true could prevent NFS data loss (see
      * http://www.pcl-developers.org/PCD-IO-consistency-on-NFS-msync-needed-td4885942.html).
      * Default: false
      * \note This option should be used by advanced users only!
      * \note Please note that using msync() on certain systems can reduce the I/O performance by up to 80%!
      * \param[in] sync set to true if msync() should be called before munmap()
      */
    void
    setMapSynchronization(bool sync)
    {
        map_synchronization_ = sync;
    }

    /** \brief Generate the header of a PCD file format
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      */
    std::string
    generateHeaderBinary(const sensor_msgs::PointCloud2& cloud,
                         const Eigen::Vector4d& origin,
                         const Eigen::Quaterniond& orientation);

    /** \brief Generate the header of a BINARY_COMPRESSED PCD file format
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      */
    std::string
    generateHeaderBinaryCompressed(const sensor_msgs::PointCloud2& cloud,
                                   const Eigen::Vector4d& origin,
                                   const Eigen::Quaterniond& orientation);

    /** \brief Generate the header of a PCD file format
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      */
    std::string
    generateHeaderASCII(const sensor_msgs::PointCloud2& cloud,
                        const Eigen::Vector4d& origin,
                        const Eigen::Quaterniond& orientation);

    /** \brief Generate the header of a PCD file format
      * \param[in] cloud the point cloud data message
      * \param[in] nr_points if given, use this to fill in WIDTH, HEIGHT (=1), and POINTS in the header
      * By default, nr_points is set to INTMAX, and the data in the header is used instead.
      */
    template <typename PointT>
    static std::string
    generateHeader(const PointCloud<PointT>& cloud,
                   const int nr_points = (std::numeric_limits<int>::max)())
    {
        std::ostringstream oss;
        oss.imbue(std::locale::classic());

        oss << "# .PCD v0.7 - Point Cloud Data file format"
            "\nVERSION 0.7"
            "\nFIELDS";

        std::vector<sensor_msgs::PointField> fields;
        getFields(cloud, fields);

        std::stringstream field_names, field_types, field_sizes, field_counts;
        for (size_t i = 0; i < fields.size(); ++i) {
            if (fields[i].name == "_")
                continue;
            // Add the regular dimension
            field_names << " " << fields[i].name;
            field_sizes << " " << getFieldSize(fields[i].datatype);
            field_types << " " << getFieldType(fields[i].datatype);
            int count = std::abs(static_cast<int>(fields[i].count));
            if (count == 0) count = 1; // check for 0 counts (coming from older converter code)
            field_counts << " " << count;
        }
        oss << field_names.str();
        oss << "\nSIZE" << field_sizes.str()
            << "\nTYPE" << field_types.str()
            << "\nCOUNT" << field_counts.str();
        // If the user passes in a number of points value, use that instead
        if (nr_points != std::numeric_limits<int>::max())
            oss << "\nWIDTH " << nr_points << "\nHEIGHT " << 1 << "\n";
        else
            oss << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

        oss << "VIEWPOINT " << cloud.sensor_origin_[0] << " " << cloud.sensor_origin_[1] << " " << cloud.sensor_origin_[2] << " " <<
            cloud.sensor_orientation_.w() << " " <<
            cloud.sensor_orientation_.x() << " " <<
            cloud.sensor_orientation_.y() << " " <<
            cloud.sensor_orientation_.z() << "\n";

        // If the user passes in a number of points value, use that instead
        if (nr_points != std::numeric_limits<int>::max())
            oss << "POINTS " << nr_points << "\n";
        else
            oss << "POINTS " << cloud.points.size() << "\n";

        return (oss.str());
    }

    /** \brief Generate the header of a PCD file format
      * \note This version is specialized for PointCloud<Eigen::MatrixXf> data types. 
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] cloud the point cloud data message
      * \param[in] nr_points if given, use this to fill in WIDTH, HEIGHT (=1), and POINTS in the header
      * By default, nr_points is set to INTMAX, and the data in the header is used instead.
      */
    //       std::string
    //       generateHeaderEigen (const PointCloud<Eigen::MatrixXf> &cloud, 
    //                            const int nr_points = (std::numeric_limits<int>::max) ());

    /** \brief Save point cloud data to a PCD file containing n-D points, in ASCII format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      * \param[in] precision the specified output numeric stream precision (default: 8)
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      *
      * As an intermediary solution, precision 8 is used, which guarantees lossless storage for RGB.
      */
    int
    writeASCII(const std::string& file_name, const sensor_msgs::PointCloud2& cloud,
               const Eigen::Vector4d& origin = Eigen::Vector4d::Zero(),
               const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity(),
               const int precision = 8);

    /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      */
    int
    writeBinary(const std::string& file_name, const sensor_msgs::PointCloud2& cloud,
                const Eigen::Vector4d& origin = Eigen::Vector4d::Zero(),
                const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity());

    /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY_COMPRESSED format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      */
    int
    writeBinaryCompressed(const std::string& file_name, const sensor_msgs::PointCloud2& cloud,
                          const Eigen::Vector4d& origin = Eigen::Vector4d::Zero(),
                          const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity());

    /** \brief Save point cloud data to a PCD file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      * \param[in] binary set to true if the file is to be written in a binary
      * PCD format, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      *
      * As an intermediary solution, precision 8 is used, which guarantees lossless storage for RGB.
      */
    inline int
    write(const std::string& file_name, const sensor_msgs::PointCloud2& cloud,
          const Eigen::Vector4d& origin = Eigen::Vector4d::Zero(),
          const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity(),
          const bool binary = false)
    {
        if (binary)
            return (writeBinary(file_name, cloud, origin, orientation));
        else
            return (writeASCII(file_name, cloud, origin, orientation, 8));
    }

    /** \brief Save point cloud data to a PCD file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message (boost shared pointer)
      * \param[in] binary set to true if the file is to be written in a binary PCD format, 
      * false (default) for ASCII
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      */
    inline int
    write(const std::string& file_name, const sensor_msgs::PointCloud2::ConstPtr& cloud,
          const Eigen::Vector4d& origin = Eigen::Vector4d::Zero(),
          const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity(),
          const bool binary = false)
    {
        return (write(file_name, *cloud, origin, orientation, binary));
    }

    /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      */
    template <typename PointT>
    int
    writeBinary(const std::string& file_name,
                const PointCloud<PointT>& cloud)
    {
        if (cloud.empty()) {
            std::cout << "Input point cloud has no data" << std::endl;
            throw IOException ("[pcl::PCDWriter::writeBinary] Input point cloud has no data!");
            return (-1);
        }
        int data_idx = 0;
        std::ostringstream oss;
        oss << generateHeader<PointT>(cloud) << "DATA binary\n";
        oss.flush();
        data_idx = static_cast<int>(oss.tellp());

#if _WIN32
        HANDLE h_native_file = CreateFileA(file_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (h_native_file == INVALID_HANDLE_VALUE) {
            std::cout << "Error during CreateFile" << std::endl;
            throw IOException ("[pcl::PCDWriter::writeBinary] Error during CreateFile!");
            return (-1);
        }
#else
        int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
        if (fd < 0) {
            throw IOException ("[pcl::PCDWriter::writeBinary] Error during open!");
            return (-1);
        }
#endif

        std::vector<sensor_msgs::PointField> fields;
        std::vector<int> fields_sizes;
        size_t fsize = 0;
        size_t data_size = 0;
        size_t nri = 0;
        getFields(cloud, fields);
        // Compute the total size of the fields
        for (size_t i = 0; i < fields.size(); ++i) {
            if (fields[i].name == "_")
                continue;

            int fs = fields[i].count * getFieldSize(fields[i].datatype);
            fsize += fs;
            fields_sizes.push_back(fs);
            fields[nri++] = fields[i];
        }
        fields.resize(nri);

        data_size = cloud.points.size() * fsize;

        // Prepare the map
#if _WIN32
        HANDLE fm = CreateFileMappingA(h_native_file, NULL, PAGE_READWRITE, 0, (DWORD) (data_idx + data_size), NULL);
        char* map = static_cast<char*>(MapViewOfFile(fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + data_size));
        CloseHandle(fm);

#else
        // Stretch the file size to the size of the data
      int result = static_cast<int> (pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET));
      if (result < 0) {
          pcl_close (fd);
          throw IOException ("[pcl::PCDWriter::writeBinary] Error during lseek ()!");
          return (-1);
      }
        // Write a bogus entry so that the new file size comes in effect
      result = static_cast<int> (::write (fd, "", 1));
      if (result != 1)
      {
          pcl_close (fd);
          throw IOException ("[pcl::PCDWriter::writeBinary] Error during write ()!");
          return (-1);
      }

      char *map = static_cast<char*> (mmap (0, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
      if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
      {
          pcl_close (fd);
          throw IOException ("[pcl::PCDWriter::writeBinary] Error during mmap ()!");
          return (-1);
      }
#endif

        // Copy the header
        memcpy(&map[0], oss.str().c_str(), data_idx);

        // Copy the data
        char* out = &map[0] + data_idx;
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            int nrj = 0;
            for (size_t j = 0; j < fields.size(); ++j) {
                memcpy(out, reinterpret_cast<const char*>(&cloud.points[i]) + fields[j].offset, fields_sizes[nrj]);
                out += fields_sizes[nrj++];
            }
        }

        // If the user set the synchronization flag on, call msync
#if !_WIN32
      if (map_synchronization_)
          msync (map, data_idx + data_size, MS_SYNC);
#endif

        // Unmap the pages of memory
#if _WIN32
        UnmapViewOfFile(map);
#else
      if (munmap (map, (data_idx + data_size)) == -1)
      {
          pcl_close (fd);
          std::cout << "error during munmap ()!" << std::endl;
          throw IOException ("[pcl::PCDWriter::writeBinary] Error during munmap ()!");
          return (-1);
      }
#endif
        // Close file
#if _WIN32
        CloseHandle(h_native_file);
#else
      pcl_close (fd);
#endif
        return (0);
    }

    /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
      * \note This version is specialized for PointCloud<Eigen::MatrixXf> data types. 
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data
      */
    //       int 
    //       writeBinaryEigen (const std::string &file_name, 
    //                         const PointCloud<Eigen::MatrixXf> &cloud);

    /** \brief Save point cloud data to a binary comprssed PCD file
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      */
    template <typename PointT>
    int
    writeBinaryCompressed(const std::string& file_name,
                          const PointCloud<PointT>& cloud)
    {
        if (cloud.points.empty()) {
            throw IOException ("[pcl::PCDWriter::writeBinaryCompressed] Input point cloud has no data!");
            return (-1);
        }
        int data_idx = 0;
        std::ostringstream oss;
        oss << generateHeader<PointT>(cloud) << "DATA binary_compressed\n";
        oss.flush();
        data_idx = static_cast<int>(oss.tellp());

#if _WIN32
        HANDLE h_native_file = CreateFileA(file_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (h_native_file == INVALID_HANDLE_VALUE) {
            throw IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during CreateFile!");
            return (-1);
        }
#else
      int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
      if (fd < 0)
      {
        throw IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during open!");
          return (-1);
      }
#endif

        std::vector<sensor_msgs::PointField> fields;
        size_t fsize = 0;
        size_t data_size = 0;
        size_t nri = 0;
        getFields(cloud, fields);
        std::vector<int> fields_sizes(fields.size());
        // Compute the total size of the fields
        for (size_t i = 0; i < fields.size(); ++i) {
            if (fields[i].name == "_")
                continue;

            fields_sizes[nri] = fields[i].count * getFieldSize(fields[i].datatype);
            fsize += fields_sizes[nri];
            fields[nri] = fields[i];
            ++nri;
        }
        fields_sizes.resize(nri);
        fields.resize(nri);

        // Compute the size of data
        data_size = cloud.points.size() * fsize;

        //////////////////////////////////////////////////////////////////////
        // Empty array holding only the valid data
        // data_size = nr_points * point_size 
        //           = nr_points * (sizeof_field_1 + sizeof_field_2 + ... sizeof_field_n)
        //           = sizeof_field_1 * nr_points + sizeof_field_2 * nr_points + ... sizeof_field_n * nr_points
        char* only_valid_data = static_cast<char*>(malloc(data_size));

        // Convert the XYZRGBXYZRGB structure to XXYYZZRGBRGB to aid compression. For
        // this, we need a vector of fields.size () (4 in this case), which points to
        // each individual plane:
        //   pters[0] = &only_valid_data[offset_of_plane_x];
        //   pters[1] = &only_valid_data[offset_of_plane_y];
        //   pters[2] = &only_valid_data[offset_of_plane_z];
        //   pters[3] = &only_valid_data[offset_of_plane_RGB];
        //
        std::vector<char*> pters(fields.size());
        int toff = 0;
        for (size_t i = 0; i < pters.size(); ++i) {
            pters[i] = &only_valid_data[toff];
            toff += fields_sizes[i] * static_cast<int>(cloud.points.size());
        }

        // Go over all the points, and copy the data in the appropriate places
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            for (size_t j = 0; j < fields.size(); ++j) {
                memcpy(pters[j], reinterpret_cast<const char*>(&cloud.points[i]) + fields[j].offset, fields_sizes[j]);
                // Increment the pointer
                pters[j] += fields_sizes[j];
            }
        }

        char* temp_buf = static_cast<char*>(malloc(static_cast<size_t>(static_cast<float>(data_size) * 1.5f + 8.0f)));
        // Compress the valid data
        unsigned int compressed_size = lzfCompress(only_valid_data,
                                                   static_cast<uint32_t>(data_size),
                                                   &temp_buf[8],
                                                   static_cast<uint32_t>(static_cast<float>(data_size) * 1.5f));
        unsigned int compressed_final_size = 0;
        // Was the compression successful?
        if (compressed_size) {
            char* header = &temp_buf[0];
            memcpy(&header[0], &compressed_size, sizeof (unsigned int));
            memcpy(&header[4], &data_size, sizeof (unsigned int));
            data_size = compressed_size + 8;
            compressed_final_size = static_cast<uint32_t>(data_size) + data_idx;
        } else {
#if !_WIN32
          pcl_close (fd);
#endif
            throw IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during compression!");
            return (-1);
        }

#if !_WIN32
        // Stretch the file size to the size of the data
      int result = static_cast<int> (pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET));
      if (result < 0)
      {
          pcl_close (fd);
          throw IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during lseek ()!");
          return (-1);
      }
        // Write a bogus entry so that the new file size comes in effect
      result = static_cast<int> (::write (fd, "", 1));
      if (result != 1)
      {
          pcl_close (fd);
          throw IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during write ()!");
          return (-1);
      }
#endif

        // Prepare the map
#if _WIN32
        HANDLE fm = CreateFileMapping(h_native_file, NULL, PAGE_READWRITE, 0, compressed_final_size, NULL);
        char* map = static_cast<char*>(MapViewOfFile(fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, compressed_final_size));
        CloseHandle(fm);

#else
      char *map = static_cast<char*> (mmap (0, compressed_final_size, PROT_WRITE, MAP_SHARED, fd, 0));
      if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
      {
          pcl_close (fd);
          throw IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during mmap ()!");
          return (-1);
      }
#endif

        // Copy the header
        memcpy(&map[0], oss.str().c_str(), data_idx);
        // Copy the compressed data
        memcpy(&map[data_idx], temp_buf, data_size);

#if !_WIN32
        // If the user set the synchronization flag on, call msync
      if (map_synchronization_)
          msync (map, compressed_final_size, MS_SYNC);
#endif

        // Unmap the pages of memory
#if _WIN32
        UnmapViewOfFile(map);
#else
      if (munmap (map, (compressed_final_size)) == -1)
      {
          pcl_close (fd);
        //throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during munmap ()!");
          return (-1);
      }
#endif
        // Close file
#if _WIN32
        CloseHandle(h_native_file);
#else
      pcl_close (fd);
#endif

        free(only_valid_data);
        free(temp_buf);
        return (0);
    }

    /** \brief Save point cloud data to a binary comprssed PCD file.
      * \note This version is specialized for PointCloud<Eigen::MatrixXf> data types. 
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      */
    //       int 
    //       writeBinaryCompressedEigen (const std::string &file_name, 
    //                                   const PointCloud<Eigen::MatrixXf> &cloud);

    /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] indices the set of point indices that we want written to disk
      */
    template <typename PointT>
    int
    writeBinary(const std::string& file_name,
                const PointCloud<PointT>& cloud,
                const std::vector<int>& indices)
    {
        if (cloud.points.empty() || indices.empty()) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Input point cloud has no data or empty indices given!");
            return (-1);
        }
        int data_idx = 0;
        std::ostringstream oss;
        oss << generateHeader<PointT>(cloud, static_cast<int>(indices.size())) << "DATA binary\n";
        oss.flush();
        data_idx = static_cast<int>(oss.tellp());

#if _WIN32
        HANDLE h_native_file = CreateFileA(file_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (h_native_file == INVALID_HANDLE_VALUE) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during CreateFile!");
            return (-1);
        }
#else
      int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
      if (fd < 0)
      {
        //throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during open!");
          return (-1);
      }
#endif

        std::vector<sensor_msgs::PointField> fields;
        std::vector<int> fields_sizes;
        size_t fsize = 0;
        size_t data_size = 0;
        size_t nri = 0;
        getFields(cloud, fields);
        // Compute the total size of the fields
        for (size_t i = 0; i < fields.size(); ++i) {
            if (fields[i].name == "_")
                continue;

            int fs = fields[i].count * getFieldSize(fields[i].datatype);
            fsize += fs;
            fields_sizes.push_back(fs);
            fields[nri++] = fields[i];
        }
        fields.resize(nri);

        data_size = indices.size() * fsize;

        // Prepare the map
#if _WIN32
        HANDLE fm = CreateFileMapping(h_native_file, NULL, PAGE_READWRITE, 0, data_idx + data_size, NULL);
        char* map = static_cast<char*>(MapViewOfFile(fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + data_size));
        CloseHandle(fm);

#else
        // Stretch the file size to the size of the data
      int result = static_cast<int> (pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET));
      if (result < 0)
      {
          pcl_close (fd);
        //throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during lseek ()!");
          return (-1);
      }
        // Write a bogus entry so that the new file size comes in effect
      result = static_cast<int> (::write (fd, "", 1));
      if (result != 1)
      {
          pcl_close (fd);
        //throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during write ()!");
          return (-1);
      }

      char *map = static_cast<char*> (mmap (0, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
      if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
      {
          pcl_close (fd);
        //throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during mmap ()!");
          return (-1);
      }
#endif

        // Copy the header
        memcpy(&map[0], oss.str().c_str(), data_idx);

        char* out = &map[0] + data_idx;
        // Copy the data
        for (size_t i = 0; i < indices.size(); ++i) {
            int nrj = 0;
            for (size_t j = 0; j < fields.size(); ++j) {
                memcpy(out, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[j].offset, fields_sizes[nrj]);
                out += fields_sizes[nrj++];
            }
        }

#if !_WIN32
        // If the user set the synchronization flag on, call msync
      if (map_synchronization_)
          msync (map, data_idx + data_size, MS_SYNC);
#endif

        // Unmap the pages of memory
#if _WIN32
        UnmapViewOfFile(map);
#else
      if (munmap (map, (data_idx + data_size)) == -1)
      {
          pcl_close (fd);
        //throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during munmap ()!");
          return (-1);
      }
#endif
        // Close file
#if _WIN32
        CloseHandle(h_native_file);
#else
      pcl_close (fd);
#endif
        return (0);
    }

    /** \brief Save point cloud data to a PCD file containing n-D points, in ASCII format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] precision the specified output numeric stream precision (default: 8)
      */
    template <typename PointT>
    int
    writeASCII(const std::string& file_name,
               const PointCloud<PointT>& cloud,
               const int precision = 8)
    {
        if (cloud.empty()) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Input point cloud has no data!");
            return (-1);
        }

        if (cloud.width * cloud.height != cloud.points.size()) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Number of points different than width * height!");
            return (-1);
        }

        std::ofstream fs;
        fs.open(file_name.c_str()); // Open file

        if (!fs.is_open() || fs.fail()) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Could not open file for writing!");
            return (-1);
        }

        fs.precision(precision);
        fs.imbue(std::locale::classic());

        std::vector<sensor_msgs::PointField> fields;
        getFields(cloud, fields);

        // Write the header information
        fs << generateHeader<PointT>(cloud) << "DATA ascii\n";

        std::ostringstream stream;
        stream.precision(precision);
        stream.imbue(std::locale::classic());
        // Iterate through the points
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            for (size_t d = 0; d < fields.size(); ++d) {
                // Ignore invalid padded dimensions that are inherited from binary data
                if (fields[d].name == "_")
                    continue;

                int count = fields[d].count;
                if (count == 0)
                    count = 1; // we simply cannot tolerate 0 counts (coming from older converter code)

                for (int c = 0; c < count; ++c) {
                    switch (fields[d].datatype) {
                    case sensor_msgs::PointField::INT8:
                        {
                            int8_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (int8_t), sizeof (int8_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::UINT8:
                        {
                            uint8_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (uint8_t), sizeof (uint8_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::INT16:
                        {
                            int16_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (int16_t), sizeof (int16_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<int16_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::UINT16:
                        {
                            uint16_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (uint16_t), sizeof (uint16_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint16_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::INT32:
                        {
                            int32_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (int32_t), sizeof (int32_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<int32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::UINT32:
                        {
                            uint32_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (uint32_t), sizeof (uint32_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::FLOAT32:
                        {
                            float value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (float), sizeof (float));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<float>(value);
                            break;
                        }
                    case sensor_msgs::PointField::FLOAT64:
                        {
                            double value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[i]) + fields[d].offset + c * sizeof (double), sizeof (double));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<double>(value);
                            break;
                        }
                    default:
                        //PCL_WARN ("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n", fields[d].datatype);
                        break;
                    }

                    if (d < fields.size() - 1 || c < static_cast<int>(fields[d].count - 1))
                        stream << " ";
                }
            }
            // Copy the stream, trim it, and write it to disk
            std::string result = stream.str();
            boost::trim(result);
            stream.str("");
            fs << result << "\n";
        }
        fs.close(); // Close file
        return (0);
    }

    /** \brief Save point cloud data to a PCD file containing n-D points, in ASCII format
      * \note This version is specialized for PointCloud<Eigen::MatrixXf> data types. 
      * \attention The PCD data is \b always stored in ROW major format! The
      * read/write PCD methods will detect column major input and automatically convert it.
      *
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] precision the specified output numeric stream precision (default: 8)
      */
    //       int 
    //       writeASCIIEigen (const std::string &file_name, 
    //                        const PointCloud<Eigen::MatrixXf> &cloud,
    //                        const int precision = 8);

    /** \brief Save point cloud data to a PCD file containing n-D points, in ASCII format
     * \param[in] file_name the output file name
     * \param[in] cloud the point cloud data message
     * \param[in] indices the set of point indices that we want written to disk
     * \param[in] precision the specified output numeric stream precision (default: 8)
     */
    template <typename PointT>
    int
    writeASCII(const std::string& file_name,
               const PointCloud<PointT>& cloud,
               const std::vector<int>& indices,
               const int precision = 8)
    {
        if (cloud.points.empty() || indices.empty()) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Input point cloud has no data or empty indices given!");
            return (-1);
        }

        if (cloud.width * cloud.height != cloud.points.size()) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Number of points different than width * height!");
            return (-1);
        }

        std::ofstream fs;
        fs.open(file_name.c_str()); // Open file
        if (!fs.is_open() || fs.fail()) {
            //throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Could not open file for writing!");
            return (-1);
        }
        fs.precision(precision);
        fs.imbue(std::locale::classic());

        std::vector<sensor_msgs::PointField> fields;
        getFields(cloud, fields);

        // Write the header information
        fs << generateHeader<PointT>(cloud, static_cast<int>(indices.size())) << "DATA ascii\n";

        std::ostringstream stream;
        stream.precision(precision);
        stream.imbue(std::locale::classic());

        // Iterate through the points
        for (size_t i = 0; i < indices.size(); ++i) {
            for (size_t d = 0; d < fields.size(); ++d) {
                // Ignore invalid padded dimensions that are inherited from binary data
                if (fields[d].name == "_")
                    continue;

                int count = fields[d].count;
                if (count == 0)
                    count = 1; // we simply cannot tolerate 0 counts (coming from older converter code)

                for (int c = 0; c < count; ++c) {
                    switch (fields[d].datatype) {
                    case sensor_msgs::PointField::INT8:
                        {
                            int8_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (int8_t), sizeof (int8_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::UINT8:
                        {
                            uint8_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (uint8_t), sizeof (uint8_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::INT16:
                        {
                            int16_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (int16_t), sizeof (int16_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<int16_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::UINT16:
                        {
                            uint16_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (uint16_t), sizeof (uint16_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint16_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::INT32:
                        {
                            int32_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (int32_t), sizeof (int32_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<int32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::UINT32:
                        {
                            uint32_t value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (uint32_t), sizeof (uint32_t));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<uint32_t>(value);
                            break;
                        }
                    case sensor_msgs::PointField::FLOAT32:
                        {
                            float value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (float), sizeof (float));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<float>(value);
                            break;
                        }
                    case sensor_msgs::PointField::FLOAT64:
                        {
                            double value;
                            memcpy(&value, reinterpret_cast<const char*>(&cloud.points[indices[i]]) + fields[d].offset + c * sizeof (double), sizeof (double));
                            if (pcl_isnan (value))
                                stream << "nan";
                            else
                                stream << boost::numeric_cast<double>(value);
                            break;
                        }
                    default:
                        //PCL_WARN ("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n", fields[d].datatype);
                        break;
                    }

                    if (d < fields.size() - 1 || c < static_cast<int>(fields[d].count - 1))
                        stream << " ";
                }
            }
            // Copy the stream, trim it, and write it to disk
            std::string result = stream.str();
            boost::trim(result);
            stream.str("");
            fs << result << "\n";
        }
        fs.close(); // Close file
        return (0);
    }

    /** \brief Save point cloud data to a PCD file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the PointCloud data
      * \param[in] binary set to true if the file is to be written in a binary
      * PCD format, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      */
    template <typename PointT>
    inline int
    write(const std::string& file_name,
          const PointCloud<PointT>& cloud,
          const bool binary = false)
    {
        if (binary)
            return (writeBinary<PointT>(file_name, cloud));
        else
            return (writeASCII<PointT>(file_name, cloud));
    }

    /** \brief Save point cloud data to a PCD file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the PointCloud data
      * \param[in] indices the set of point indices that we want written to disk
      * \param[in] binary set to true if the file is to be written in a binary
      * PCD format, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      */
    template <typename PointT>
    inline int
    write(const std::string& file_name,
          const PointCloud<PointT>& cloud,
          const std::vector<int>& indices,
          bool binary = false)
    {
        if (binary)
            return (writeBinary<PointT>(file_name, cloud, indices));
        else
            return (writeASCII<PointT>(file_name, cloud, indices));
    }

private:
    /** \brief Set to true if msync() should be called before munmap(). Prevents data loss on NFS systems. */
    bool map_synchronization_;

    typedef std::pair<std::string, ChannelProperties> pair_channel_properties;

    /** \brief Internal structure used to sort the ChannelProperties in the
      * cloud.channels map based on their offset. 
      */
    struct ChannelPropertiesComparator
    {
        bool
        operator()(const pair_channel_properties& lhs, const pair_channel_properties& rhs)
        {
            return (lhs.second.offset < rhs.second.offset);
        }
    };
};

namespace io
{
/** \brief Load a PCD v.6 file into a templated PointCloud type.
  * 
  * Any PCD files > v.6 will generate a warning as a
  * sensor_msgs/PointCloud2 message cannot hold the sensor origin.
  *
  * \param[in] file_name the name of the file to load
  * \param[out] cloud the resultant templated point cloud
  * \ingroup io
  */
inline int
loadPCDFile(const std::string& file_name, sensor_msgs::PointCloud2& cloud)
{
    PCDReader p;
    return (p.read(file_name, cloud));
}

/** \brief Load any PCD file into a templated PointCloud type.
  * \param[in] file_name the name of the file to load
  * \param[out] cloud the resultant templated point cloud
  * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
  * \param[out] orientation the sensor acquisition orientation (only for >
  * PCD_V7 - identity if not present)
  * \ingroup io
  */
inline int
loadPCDFile(const std::string& file_name, sensor_msgs::PointCloud2& cloud,
            Eigen::Vector4d& origin, Eigen::Quaterniond& orientation)
{
    PCDReader p;
    int pcd_version;
    return (p.read(file_name, cloud, origin, orientation, pcd_version));
}

/** \brief Load any PCD file into a templated PointCloud type
  * \param[in] file_name the name of the file to load
  * \param[out] cloud the resultant templated point cloud
  * \ingroup io
  */
template <typename PointT>
inline int
loadPCDFile(const std::string& file_name, PointCloud<PointT>& cloud)
{
    PCDReader p;
    return (p.read(file_name, cloud));
}

/** \brief Save point cloud data to a PCD file containing n-D points
  * \param[in] file_name the output file name
  * \param[in] cloud the point cloud data message
  * \param[in] origin the sensor acquisition origin
  * \param[in] orientation the sensor acquisition orientation
  * \param[in] binary_mode true for binary mode, false (default) for ASCII
  *
  * Caution: PointCloud structures containing an RGB field have
  * traditionally used packed float values to store RGB data. Storing a
  * float as ASCII can introduce variations to the smallest bits, and
  * thus significantly alter the data. This is a known issue, and the fix
  * involves switching RGB data to be stored as a packed integer in
  * future versions of PCL.
  * \ingroup io
  */
inline int
savePCDFile(const std::string& file_name, const sensor_msgs::PointCloud2& cloud,
            const Eigen::Vector4d& origin = Eigen::Vector4d::Zero(),
            const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity(),
            const bool binary_mode = false)
{
    PCDWriter w;
    return (w.write(file_name, cloud, origin, orientation, binary_mode));
}

/** \brief Templated version for saving point cloud data to a PCD file
  * containing a specific given cloud format
  * \param[in] file_name the output file name
  * \param[in] cloud the point cloud data message
  * \param[in] binary_mode true for binary mode, false (default) for ASCII
  *
  * Caution: PointCloud structures containing an RGB field have
  * traditionally used packed float values to store RGB data. Storing a
  * float as ASCII can introduce variations to the smallest bits, and
  * thus significantly alter the data. This is a known issue, and the fix
  * involves switching RGB data to be stored as a packed integer in
  * future versions of PCL.
  * \ingroup io
  */
template <typename PointT>
inline int
savePCDFile(const std::string& file_name, const PointCloud<PointT>& cloud, bool binary_mode = true)
{
    PCDWriter w;
    return (w.write<PointT>(file_name, cloud, binary_mode));
}

/** 
  * \brief Templated version for saving point cloud data to a PCD file
  * containing a specific given cloud format.
  *
  *      This version is to retain backwards compatibility.
  * \param[in] file_name the output file name
  * \param[in] cloud the point cloud data message
  *
  * Caution: PointCloud structures containing an RGB field have
  * traditionally used packed float values to store RGB data. Storing a
  * float as ASCII can introduce variations to the smallest bits, and
  * thus significantly alter the data. This is a known issue, and the fix
  * involves switching RGB data to be stored as a packed integer in
  * future versions of PCL.
  * \ingroup io
  */
template <typename PointT>
inline int
savePCDFileASCII(const std::string& file_name, const PointCloud<PointT>& cloud)
{
    PCDWriter w;
    return (w.write<PointT>(file_name, cloud, false));
}

/** 
  * \brief Templated version for saving point cloud data to a PCD file
  * containing a specific given cloud format.
  *
  *      This version is to retain backwards compatibility.
  * \param[in] file_name the output file name
  * \param[in] cloud the point cloud data message
  * \ingroup io
  */
template <typename PointT>
inline int
savePCDFileBinary(const std::string& file_name, const PointCloud<PointT>& cloud)
{
    PCDWriter w;
    return (w.write<PointT>(file_name, cloud, true));
}

/** 
  * \brief Templated version for saving point cloud data to a PCD file
  * containing a specific given cloud format
  *
  * \param[in] file_name the output file name
  * \param[in] cloud the point cloud data message
  * \param[in] indices the set of indices to save
  * \param[in] binary_mode true for binary mode, false (default) for ASCII
  *
  * Caution: PointCloud structures containing an RGB field have
  * traditionally used packed float values to store RGB data. Storing a
  * float as ASCII can introduce variations to the smallest bits, and
  * thus significantly alter the data. This is a known issue, and the fix
  * involves switching RGB data to be stored as a packed integer in
  * future versions of PCL.
  * \ingroup io
  */
template <typename PointT>
int
savePCDFile(const std::string& file_name,
            const PointCloud<PointT>& cloud,
            const std::vector<int>& indices,
            const bool binary_mode = false)
{
    // Save the data
    PCDWriter w;
    return (w.write<PointT>(file_name, cloud, indices, binary_mode));
}
}


CLOUD_BLEND_DOUBLE_NAMESPACE_END