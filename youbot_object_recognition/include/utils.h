#ifndef UTILS_
#define UTILS_

#include <Eigen/Core>
#include "cv_ext/cv_ext.h"
#include "raster_object_model3D.h"

/*struct PoseCandidate
{
  double score;
  Eigen::Quaterniond r;
  Eigen::Vector3d t;
};*/

struct ObjectCandidate{
  Eigen::Affine3d pose;
  double avgDist;
};

struct AcquisitionData{
  cv::Mat scene_dist_map;
  Eigen::Affine3d camera_pose;
};

#endif
