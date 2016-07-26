#pragma once
#include <stdexcept>
#include <sstream>
#include "macros.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
/** \class PCLException
  * \brief A base class for all pcl exceptions which inherits from std::runtime_error
  * \author Eitan Marder-Eppstein, Suat Gedikli, Nizar Sallem
  */
class PCLException : public std::runtime_error
{
public:

    PCLException(const std::string& error_description,
                 const std::string& file_name = "",
                 const std::string& function_name = "",
                 unsigned line_number = 0) throw ()
        : std::runtime_error(error_description)
          , file_name_(file_name)
          , function_name_(function_name)
          , line_number_(line_number)
    {
    }

    virtual ~PCLException() throw ()
    {
    }

    const std::string&
    getFileName() const throw ()
    {
        return file_name_;
    }

    const std::string&
    getFunctionName() const throw ()
    {
        return function_name_;
    }

    unsigned
    getLineNumber() const throw ()
    {
        return line_number_;
    }

    std::string
    detailedMessage() const throw ()
    {
        std::stringstream sstream;
        if (function_name_ != "")
            sstream << function_name_ << " ";

        if (file_name_ != "") {
            sstream << "in " << file_name_ << " ";
            if (line_number_ != 0)
                sstream << "@ " << line_number_ << " ";
        }
        sstream << ":" << what();

        return sstream.str();
    }

protected:
    std::string file_name_;
    std::string function_name_;
    unsigned line_number_;
};

/** \class InvalidConversionException
  * \brief An exception that is thrown when a PointCloud2 message cannot be converted into a PCL type
  */
class InvalidConversionException : public PCLException
{
public:

    InvalidConversionException(const std::string& error_description,
                               const std::string& file_name = "",
                               const std::string& function_name = "",
                               unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

/** \class IsNotDenseException
  * \brief An exception that is thrown when a PointCloud is not dense but is attemped to be used as dense
  */
class IsNotDenseException : public PCLException
{
public:

    IsNotDenseException(const std::string& error_description,
                        const std::string& file_name = "",
                        const std::string& function_name = "",
                        unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

/** \class InvalidSACModelTypeException
  * \brief An exception that is thrown when a sample consensus model doesn't
  * have the correct number of samples defined in model_types.h
  */
class InvalidSACModelTypeException : public PCLException
{
public:

    InvalidSACModelTypeException(const std::string& error_description,
                                 const std::string& file_name = "",
                                 const std::string& function_name = "",
                                 unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

/** \class IOException
  * \brief An exception that is thrown during an IO error (typical read/write errors)
  */
class IOException : public PCLException
{
public:

    IOException(const std::string& error_description,
                const std::string& file_name = "",
                const std::string& function_name = "",
                unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

/** \class InitFailedException
  * \brief An exception thrown when init can not be performed should be used in all the
  * PCLBase class inheritants.
  */
class InitFailedException : public PCLException
{
public:
    InitFailedException(const std::string& error_description = "",
                        const std::string& file_name = "",
                        const std::string& function_name = "",
                        unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

/** \class UnorganizedPointCloudException
  * \brief An exception that is thrown when an organized point cloud is needed
  * but not provided.
  */
class UnorganizedPointCloudException : public PCLException
{
public:

    UnorganizedPointCloudException(const std::string& error_description,
                                   const std::string& file_name = "",
                                   const std::string& function_name = "",
                                   unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

/** \class KernelWidthTooSmallException
  * \brief An exception that is thrown when the kernel size is too small
  */
class KernelWidthTooSmallException : public PCLException
{
public:

    KernelWidthTooSmallException(const std::string& error_description,
                                 const std::string& file_name = "",
                                 const std::string& function_name = "",
                                 unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

class UnhandledPointTypeException : public PCLException
{
public:
    UnhandledPointTypeException(const std::string& error_description,
                                const std::string& file_name = "",
                                const std::string& function_name = "",
                                unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};

class ComputeFailedException : public PCLException
{
public:
    ComputeFailedException(const std::string& error_description,
                           const std::string& file_name = "",
                           const std::string& function_name = "",
                           unsigned line_number = 0) throw ()
        : PCLException(error_description, file_name, function_name, line_number)
    {
    }
};
CLOUD_BLEND_DOUBLE_NAMESPACE_END