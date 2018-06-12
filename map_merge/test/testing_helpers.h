#ifndef TESTING_HELPERS_H_
#define TESTING_HELPERS_H_

#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>

const float resolution = 0.05f;

nav_msgs::OccupancyGridConstPtr loadMap(const std::string& filename);
void saveMap(const std::string& filename,
             const nav_msgs::OccupancyGridConstPtr& map);
std::tuple<double, double, double> randomAngleTxTy();
geometry_msgs::Transform randomTransform();
cv::Mat randomTransformMatrix();

/* map_server is really bad. until there is no replacement I will implement it
 * by myself */
template <typename InputIt>
std::vector<nav_msgs::OccupancyGridConstPtr> loadMaps(InputIt filenames_begin,
                                                      InputIt filenames_end)
{
  std::vector<nav_msgs::OccupancyGridConstPtr> result;

  for (InputIt it = filenames_begin; it != filenames_end; ++it) {
    result.emplace_back(loadMap(*it));
  }
  return result;
}

nav_msgs::OccupancyGridConstPtr loadMap(const std::string& filename)
{
  cv::Mat lookUpTable(1, 256, CV_8S);
  signed char* p = lookUpTable.ptr<signed char>();
  p[254] = 0;
  p[205] = -1;
  p[0] = 100;

  cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  if (img.empty()) {
    throw std::runtime_error("could not load map");
  }
  nav_msgs::OccupancyGridPtr grid{new nav_msgs::OccupancyGrid()};
  grid->info.width = static_cast<uint>(img.size().width);
  grid->info.height = static_cast<uint>(img.size().height);
  grid->info.resolution = resolution;
  grid->data.resize(static_cast<size_t>(img.size().area()));
  cv::Mat grid_view(img.size(), CV_8S,
                    const_cast<signed char*>(grid->data.data()));
  cv::LUT(img, lookUpTable, grid_view);

  return grid;
}

void saveMap(const std::string& filename,
             const nav_msgs::OccupancyGridConstPtr& map)
{
  cv::Mat lookUpTable(1, 256, CV_8U);
  uchar* p = lookUpTable.ptr();
  for (int i = 0; i < 255; ++i) {
    if (i >= 0 && i < 10)
      p[i] = 254;
    else
      p[i] = 0;
  }
  p[255] = 205;

  cv::Mat img(map->info.height, map->info.width, CV_8S,
              const_cast<signed char*>(map->data.data()));
  cv::Mat out_img;
  cv::LUT(img, lookUpTable, out_img);
  cv::imwrite(filename, out_img);
}

std::tuple<double, double, double> randomAngleTxTy()
{
  static std::mt19937_64 g(156468754 /*magic*/);
  std::uniform_real_distribution<double> rotation_dis(0., 2 * std::acos(-1));
  std::uniform_real_distribution<double> translation_dis(-1000, 1000);

  return std::tuple<double, double, double>(rotation_dis(g), translation_dis(g),
                                            translation_dis(g));
}

geometry_msgs::Transform randomTransform()
{
  double angle, tx, ty;
  std::tie(angle, tx, ty) = randomAngleTxTy();
  tf2::Transform transform;
  tf2::Quaternion rotation;
  rotation.setEuler(0., 0., angle);
  rotation.normalize();
  transform.setRotation(rotation);
  transform.setOrigin(tf2::Vector3(tx, ty, 0.));

  auto msg = toMsg(transform);
  // normalize quaternion such that w > 0 (q and -q represents the same
  // transformation)
  if (msg.rotation.w < 0.) {
    msg.rotation.x *= -1.;
    msg.rotation.y *= -1.;
    msg.rotation.z *= -1.;
    msg.rotation.w *= -1.;
  }

  return msg;
}

cv::Mat randomTransformMatrix()
{
  double angle, tx, ty;
  std::tie(angle, tx, ty) = randomAngleTxTy();
  cv::Mat transform =
      (cv::Mat_<double>(3, 3) << std::cos(angle), -std::sin(angle), tx,
       std::sin(angle), std::cos(angle), ty, 0., 0., 1.);

  return transform;
}

static inline bool isIdentity(const geometry_msgs::Transform& transform)
{
  tf2::Transform t;
  tf2::fromMsg(transform, t);
  return tf2::Transform::getIdentity() == t;
}

static inline bool isIdentity(const geometry_msgs::Quaternion& rotation)
{
  tf2::Quaternion q;
  tf2::fromMsg(rotation, q);
  return tf2::Quaternion::getIdentity() == q;
}

// data size is consistent with height and width
static inline bool consistentData(const nav_msgs::OccupancyGrid& grid)
{
  return grid.info.width * grid.info.height == grid.data.size();
}

// ignores header, map_load_time and origin
static inline bool operator==(const nav_msgs::OccupancyGrid& grid1,
                              const nav_msgs::OccupancyGrid& grid2)
{
  bool equal = true;
  equal &= grid1.info.width == grid2.info.width;
  equal &= grid1.info.height == grid2.info.height;
  equal &= std::abs(grid1.info.resolution - grid2.info.resolution) <
           std::numeric_limits<float>::epsilon();
  equal &= grid1.data.size() == grid2.data.size();
  for (size_t i = 0; i < grid1.data.size(); ++i) {
    equal &= grid1.data[i] == grid2.data[i];
  }
  return equal;
}

#endif  // TESTING_HELPERS_H_
