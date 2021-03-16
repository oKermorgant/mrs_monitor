#include <mrs_monitor/geometry.h>

namespace mrs_monitor
{

double travelTime(const PoseStamped &prev, const PoseStamped &next, double vmax_inv, double wmax_inv)
{
  // just take Euclidean + angular time estimates

  // linear part
  const auto dx(prev.pose.position.x - next.pose.position.x);
  const auto dy(prev.pose.position.y - next.pose.position.y);

  // angular part
  const auto c1(prev.pose.orientation.w);
  const auto s1(prev.pose.orientation.z);
  const auto c2(next.pose.orientation.w);
  const auto s2(next.pose.orientation.z);

  return sqrt(dx*dx + dy*dy)*vmax_inv + 2*abs(atan2(s1*c2-c1*s2, c1*c2+s1*s2))*wmax_inv;
}

double travelTime(const std::vector<PoseStamped> &path, double vmax_inv, double wmax_inv)
{
  if(path.size() < 2)
    return 0;

  double time(0);
  auto prev(path[0]);
  for(size_t i = 1; i < path.size(); ++i)
  {
    auto &cur(path[i]);
    time += travelTime(prev, cur, vmax_inv, wmax_inv);
    prev = cur;
  }
  return time;
}

}
