#pragma once

#define DEBUG_VIEW 0
#define DEBUG_OUTPUT 0

#define CLOUD_BLEND_DOUBLE_NAMESPACE cloud_blend_double
#define CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN namespace CLOUD_BLEND_DOUBLE_NAMESPACE {
#define CLOUD_BLEND_DOUBLE_NAMESPACE_END }

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

//禁用默认的拷贝和复制构造函数
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
            TypeName(const TypeName&); \
            TypeName& operator=(const TypeName&)

//单例模式
#define SINGLETON_CLASS(class_name)\
    private:\
    class_name();\
    public:\
    static class_name & instance()\
        {\
        static class_name ins;\
        return ins;\
        }

#if _WIN32
#define pcl_isnan(x)    _isnan(x)
#define pcl_isfinite(x) (_finite(x) != 0)
#define pcl_isinf(x)    (_finite(x) == 0)
#else
#define pcl_isnan(x)    isnan(x)
#define pcl_isfinite(x) (finite(x) != 0)
#define pcl_isinf(x)    (finite(x) == 0)
#endif
