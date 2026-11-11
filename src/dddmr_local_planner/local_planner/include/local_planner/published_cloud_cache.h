/*
 * Diagnostic point-cloud publish cache.
 *
 * The perception cache replaces SharedData::aggregate_observation_ with a new
 * shared_ptr on every refresh, so holding the last published pointer is an
 * exact "unchanged since the last publish" test. The cache keeps a strong
 * reference rather than a raw address so that a recycled allocation cannot
 * masquerade as an unchanged cloud.
 */

#ifndef LOCAL_PLANNER_PUBLISHED_CLOUD_CACHE_H_
#define LOCAL_PLANNER_PUBLISHED_CLOUD_CACHE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lp
{

class PublishedCloudCache
{
public:
  bool shouldPublish(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud) const
  {
    return cloud && cloud != last_published_;
  }

  void remember(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud)
  {
    last_published_ = cloud;
  }

  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & lastPublished() const
  {
    return last_published_;
  }

private:
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr last_published_;
};

}  // namespace lp

#endif  // LOCAL_PLANNER_PUBLISHED_CLOUD_CACHE_H_
