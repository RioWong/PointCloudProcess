
#include "data_struct.h"
#include "calculate_feature.h"
#include "extraction_tree.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/thread/thread.hpp>
#include <ctime>

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
void tree_filter(CloudPtr src_cloud, CloudPtr dst_cloud);
void ground_filter(CloudPtr src_cloud, CloudPtr dst_cloud);
void shaft_filter(CloudPtr src_cloud, CloudPtr dst_cloud);

CLOUD_BLEND_DOUBLE_NAMESPACE_END