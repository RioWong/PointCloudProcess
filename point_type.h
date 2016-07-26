#pragma once
#include <ostream>
#include <Eigen/Eigen>
#include <stdint.h>
#include "macros.h"
#include "register_point_struct.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
struct EIGEN_ALIGN16 _PointXYZRGBA
{
    EIGEN_ALIGN16
    union
    {
        double data[4];

        struct
        {
            double x;
            double y;
            double z;
        };
    };

    inline Eigen::Map<Eigen::Vector3d> getVector3dMap()
    {
        return (Eigen::Vector3d::Map(data));
    }

    inline const Eigen::Map<const Eigen::Vector3d> getVector3dMap() const
    {
        return (Eigen::Vector3d::Map(data));
    }

    inline Eigen::Map<Eigen::Vector4d, Eigen::Aligned> getVector4dMap()
    {
        return (Eigen::Vector4d::MapAligned(data));
    }

    inline const Eigen::Map<const Eigen::Vector4d, Eigen::Aligned> getVector4dMap() const
    {
        return (Eigen::Vector4d::MapAligned(data));
    }

    inline Eigen::Map<Eigen::Array3d> getArray3dMap()
    {
        return (Eigen::Array3d::Map(data));
    }

    inline const Eigen::Map<const Eigen::Array3d> getArray3dMap() const
    {
        return (Eigen::Array3d::Map(data));
    }

    inline Eigen::Map<Eigen::Array4d, Eigen::Aligned> getArray4dMap()
    {
        return (Eigen::Array4d::MapAligned(data));
    }

    inline const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> getArray4dMap() const
    {
        return (Eigen::Array4d::MapAligned(data));
    }

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
    uint32_t stamp_id;    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZRGBA : public _PointXYZRGBA
{
    inline PointXYZRGBA()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        r = g = b = a = 0;
    }

    inline Eigen::Vector3i getRGBVector3i()
    {
        return (Eigen::Vector3i(r, g, b));
    }

    inline const Eigen::Vector3i getRGBVector3i() const
    {
        return (Eigen::Vector3i(r, g, b));
    }

    inline Eigen::Vector4i getRGBVector4i()
    {
        return (Eigen::Vector4i(r, g, b, 0));
    }

    inline const Eigen::Vector4i getRGBVector4i() const
    {
        return (Eigen::Vector4i(r, g, b, 0));
    }
};

inline std::ostream&
operator <<(std::ostream& os, const PointXYZRGBA& p)
{
    const unsigned char* rgba_ptr = reinterpret_cast<const unsigned char*>(&p.rgba);
    os << "(" << p.x << "," << p.y << "," << p.z << " - "
        << static_cast<int>(*rgba_ptr) << ","
        << static_cast<int>(*(rgba_ptr + 1)) << ","
        << static_cast<int>(*(rgba_ptr + 2)) << ","
        << static_cast<int>(*(rgba_ptr + 3)) << ")";
    return (os);
}

struct EIGEN_ALIGN16 _PointXYZRGBA_F
{
    EIGEN_ALIGN16
    union
    {
        float data[4];

        struct
        {
            float x;
            float y;
            float z;
        };
    };

    inline Eigen::Map<Eigen::Vector3f> getVector3fMap()
    {
        return (Eigen::Vector3f::Map(data));
    }

    inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap() const
    {
        return (Eigen::Vector3f::Map(data));
    }

    inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap()
    {
        return (Eigen::Vector4f::MapAligned(data));
    }

    inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4dMap() const
    {
        return (Eigen::Vector4f::MapAligned(data));
    }

    inline Eigen::Map<Eigen::Array3f> getArray3fMap()
    {
        return (Eigen::Array3f::Map(data));
    }

    inline const Eigen::Map<const Eigen::Array3f> getArray3fMap() const
    {
        return (Eigen::Array3f::Map(data));
    }

    inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap()
    {
        return (Eigen::Array4f::MapAligned(data));
    }

    inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap() const
    {
        return (Eigen::Array4f::MapAligned(data));
    }

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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZRGBA_F : public _PointXYZRGBA_F
{
    inline PointXYZRGBA_F()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        r = g = b = a = 0;
    }

    inline Eigen::Vector3i getRGBVector3i()
    {
        return (Eigen::Vector3i(r, g, b));
    }

    inline const Eigen::Vector3i getRGBVector3i() const
    {
        return (Eigen::Vector3i(r, g, b));
    }

    inline Eigen::Vector4i getRGBVector4i()
    {
        return (Eigen::Vector4i(r, g, b, 0));
    }

    inline const Eigen::Vector4i getRGBVector4i() const
    {
        return (Eigen::Vector4i(r, g, b, 0));
    }
};

inline std::ostream&
operator <<(std::ostream& os, const PointXYZRGBA_F& p)
{
    const unsigned char* rgba_ptr = reinterpret_cast<const unsigned char*>(&p.rgba);
    os << "(" << p.x << "," << p.y << "," << p.z << " - "
        << static_cast<int>(*rgba_ptr) << ","
        << static_cast<int>(*(rgba_ptr + 1)) << ","
        << static_cast<int>(*(rgba_ptr + 2)) << ","
        << static_cast<int>(*(rgba_ptr + 3)) << ")";
    return (os);
}

struct EIGEN_ALIGN16 _PointC
{
    EIGEN_ALIGN16
    union
    {
        double data[4];

        struct
        {
            double x;
            double y;
            double z;
        };
    };

    inline Eigen::Map<Eigen::Vector3d> getVector3dMap()
    {
        return (Eigen::Vector3d::Map(data));
    }

    inline const Eigen::Map<const Eigen::Vector3d> getVector3dMap() const
    {
        return (Eigen::Vector3d::Map(data));
    }

    inline Eigen::Map<Eigen::Vector4d, Eigen::Aligned> getVector4dMap()
    {
        return (Eigen::Vector4d::MapAligned(data));
    }

    inline const Eigen::Map<const Eigen::Vector4d, Eigen::Aligned> getVector4dMap() const
    {
        return (Eigen::Vector4d::MapAligned(data));
    }

    inline Eigen::Map<Eigen::Array3d> getArray3dMap()
    {
        return (Eigen::Array3d::Map(data));
    }

    inline const Eigen::Map<const Eigen::Array3d> getArray3dMap() const
    {
        return (Eigen::Array3d::Map(data));
    }

    inline Eigen::Map<Eigen::Array4d, Eigen::Aligned> getArray4dMap()
    {
        return (Eigen::Array4d::MapAligned(data));
    }

    inline const Eigen::Map<const Eigen::Array4d, Eigen::Aligned> getArray4dMap() const
    {
        return (Eigen::Array4d::MapAligned(data));
    }

    uint64_t stamp;
    double angle_z;
    double distance_sqr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointC : public _PointC
{
    inline PointC()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        angle_z = 0;
        distance_sqr = 0;
    }
};

inline std::ostream&
operator <<(std::ostream& os, const PointC& p)
{
    os << "(" << p.x << "," << p.y << "," << p.z << ")";
    return (os);
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END
//template <typename PointT> struct cloud_blend_double::traits::fieldList;

POINT_CLOUD_REGISTER_POINT_STRUCT (CLOUD_BLEND_DOUBLE_NAMESPACE::_PointXYZRGBA,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (uint32_t, rgba, rgba)
	(uint32_t,stamp_id,stamp_id)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(CLOUD_BLEND_DOUBLE_NAMESPACE::PointXYZRGBA, CLOUD_BLEND_DOUBLE_NAMESPACE::_PointXYZRGBA)

POINT_CLOUD_REGISTER_POINT_STRUCT (CLOUD_BLEND_DOUBLE_NAMESPACE::_PointXYZRGBA_F,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, rgba, rgba)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(CLOUD_BLEND_DOUBLE_NAMESPACE::PointXYZRGBA_F, CLOUD_BLEND_DOUBLE_NAMESPACE::_PointXYZRGBA_F)

POINT_CLOUD_REGISTER_POINT_STRUCT (CLOUD_BLEND_DOUBLE_NAMESPACE::_PointC,
    (double, x, x)
    (double, y, y)
    (double, z, z)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(CLOUD_BLEND_DOUBLE_NAMESPACE::PointC, CLOUD_BLEND_DOUBLE_NAMESPACE::_PointC)
