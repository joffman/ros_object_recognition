/** \file
 * \brief Implementation of the FeatureDetector class.
 */

/*
 * TODO: Right now, selectionIsValid() and lookingForTarget() represent
 *  more or less the same thing. We need to check the quality of the
 *  computed features to get a meaningful difference.
 *  First we check the selected area. Only when the selection-area is non-
 *  empty, we have a valid selection.
 *  After that (and everytime the surf-parameters are changed), we compute
 *  the target-features (keypoints and descriptors). We have to check them
 *  (e.g. at least min_num_matches features), and only if they are valid,
 *  we are 'lookingForTarget'.
 *  In summary:
 *    selectionIsValid means that the user has selected a non-empty area.
 *    lookingForTarget means that we have computed target features from
 *      the valid selection, that make it possible to detect the target.
 * TODO: Our use of the param_manager_ is probably a violation of the
 *  'law of demeter'. Is there a better way?
 */

// std.
#include <string>
#include <stdexcept>  // runtime_error
#include <vector>
#include <algorithm>  // copy_if()
#include <utility>    // pair
#include <tuple>      // tie()

// OpenCV.
#include <opencv2/core/core.hpp>        // cv-types, perspectiveTransform()
#include <opencv2/calib3d/calib3d.hpp>  // findHomography()

// ROS.
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

// object_detection_*.
#include <object_detection_2d_msgs/DetectedObject2D.h>
#include <object_detection_2d_msgs/DetectedObject2DArray.h>

// Headers of this package.
#include <feature_detector/feature_detector.h>
#include <feature_detector/parameter_manager.h>

using object_detection_2d_msgs::DetectedObject2D;
using object_detection_2d_msgs::DetectedObject2DArray;
using namespace std;


namespace {

inline vector<cv::DMatch> filterMatches(const vector<cv::DMatch>& matches,
                                        double max_match_distance)
{
    vector<cv::DMatch> filtered_matches;
    copy_if(matches.begin(), matches.end(),
            back_inserter(filtered_matches),
            [&max_match_distance](const cv::DMatch& dmatch)
            { return dmatch.distance <= max_match_distance; });
    return filtered_matches;
}


inline feature_detector::MatchedPoints extractMatchedPoints(
    const vector<cv::DMatch>& matches,
    const vector<cv::KeyPoint>& target_keypts,
    const vector<cv::KeyPoint>& query_keypts)
{
  feature_detector::MatchedPoints mp;
  mp.target_pts.reserve(matches.size());
  mp.query_pts.reserve(matches.size());

  for_each(matches.begin(), matches.end(),
           [&](const cv::DMatch& m)
           { mp.target_pts.push_back(target_keypts[m.trainIdx].pt);
           mp.query_pts.push_back(query_keypts[m.queryIdx].pt); });

  return mp;
}


// TODO: I think this function is already implemented in binary_detector.
//  Maybe move it into some general package.
inline geometry_msgs::Point32 cvPoint2rosPoint(const cv::Point2f& cv_pt)
{
  geometry_msgs::Point32 ros_pt;

  ros_pt.x = cv_pt.x;
  ros_pt.y = cv_pt.y;
  ros_pt.z = 0.;

  return ros_pt;
}


inline geometry_msgs::Polygon cvPolygon2rosPolygon(
    const vector<cv::Point2f>& cv_poly)
{
  geometry_msgs::Polygon ros_poly;

  ros_poly.points.reserve(cv_poly.size());
  for (const auto& cv_pt : cv_poly)
    ros_poly.points.push_back(cvPoint2rosPoint(cv_pt));

  return ros_poly;
}


inline vector<cv::Point2f> rect2points(const cv::Rect& rect)
{
  vector<cv::Point2f> rect_pts {
    rect.tl(),
    cv::Point2f(rect.tl().x, rect.br().y),
    rect.br(),
    cv::Point2f(rect.br().x, rect.tl().y)};
  return rect_pts;
}


inline vector<cv::Point2f> transformRectangle(const cv::Rect& rect,
                                              const cv::Mat& homography)
{
  vector<cv::Point2f> rect_pts {rect2points(rect)};

  vector<cv::Point2f> transformed_pts;
  cv::perspectiveTransform(rect_pts, transformed_pts, homography);

  return transformed_pts;
}


inline geometry_msgs::Polygon transformRectangleIntoPolygon(
    const cv::Rect& rect,       // transform this rectangle
    const cv::Mat& homography)  // using this homography matrix
{
  vector<cv::Point2f> transformed_pts {transformRectangle(rect, homography)};
  geometry_msgs::Polygon poly {cvPolygon2rosPolygon(transformed_pts)};
  return poly;
}


inline DetectedObject2D createDetectedObject2D(
    const string& name, const geometry_msgs::Polygon& polygon)
{
  DetectedObject2D obj;
  obj.name = name;
  obj.polygon = polygon;
  return obj;
}

} // anonymous namespace


namespace feature_detector {

FeatureDetector::FeatureDetector(ParameterManager* pm)
    : param_manager_ {pm}
{
  if (!pm)
    throw runtime_error {"FeatureDetector: nullptr passed to constructor"};
}


void FeatureDetector::processAreaSelection(const cv::Rect& area)
{
  param_manager_->processAreaSelection(area);
}


DetectedObject2DArray FeatureDetector::detect(const cv::Mat& image)
{
  param_manager_->storeImage(image);

  DetectedObject2DArray objects;
  if (lookingForTarget()) {
    // C++17: auto [obj, object_found] = tryDetectObject(image);
    DetectedObject2D obj;
    bool object_found;
    tie(obj, object_found) = tryDetectObject(image);
    if (object_found)
      objects.objects.push_back(obj);
  }
  fillObjectsHeader(objects);
  return objects;
}


pair<DetectedObject2D, bool> FeatureDetector::tryDetectObject(
    const cv::Mat& image) const
{
  // C++17: auto [keypoints, descriptors] = computeFeatures(image);
  vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  param_manager_->surf()->detectAndCompute(image, cv::noArray(),
                                         keypoints, descriptors);

  vector<cv::DMatch> matches;
  param_manager_->matcher()->match(descriptors, matches);
  vector<cv::DMatch> good_matches {
    filterMatches(matches, param_manager_->matchDistanceThreshold())};

  DetectedObject2D object;
  bool object_found {false};
  try {
    object = detectObject(good_matches, keypoints);
    object_found = true;
  }
  catch (const cv::Exception& e) {
    ;
  }

  return make_pair(object, object_found);
}


DetectedObject2D FeatureDetector::detectObject(
    const vector<cv::DMatch>& matches,
    const vector<cv::KeyPoint>& query_keypoints) const
{
  MatchedPoints matched_points {extractMatchedPoints(
          matches, param_manager_->targetKeypoints(), query_keypoints)};

  cv::Mat homography = cv::findHomography(
      matched_points.target_pts,
      matched_points.query_pts,
      cv::RANSAC, // this one is said to be robust against outliers
      param_manager_->ransacReprojThreshold());

  geometry_msgs::Polygon object_polygon {
    transformRectangleIntoPolygon(param_manager_->selectionArea(), homography)};

  return createDetectedObject2D(param_manager_->objectName(), object_polygon);
}


void FeatureDetector::fillObjectsHeader(DetectedObject2DArray& objects) const
{
  objects.header.stamp = ros::Time::now();
  objects.header.frame_id = "camera_rgb_frame";
}

} // feature_detector
