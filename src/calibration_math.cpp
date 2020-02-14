/**
*
* \author     Monika Roznere <monika.roznere.gr@dartmouth.edu> <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "escalibr/calibration_math.h"

#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include <math.h>

// #include <ros/package.h>

namespace escalibr
{

// TO DO: CURRENTLY NOT USED
// struct AppEllipse
// {
//   bool operator() (Eigen::Vector3f pt1, Eigen::Vector3f pt2) {return (pt1[2] < pt2[2]);}
// }
// appEllipse;


bool CalibrationMath::detectEdgePoint(float distance, int confidence, cv::KeyPoint sphere)
{
  // Sphere is detected in image
  if (sphere.size != 0)
  {
    // Setting up to solve PnP problem
    std::vector<cv::Point2d> image_points;
    image_points.push_back(cv::Point2d(sphere.pt.x - sphere.size/2, sphere.pt.y));
    image_points.push_back(cv::Point2d(sphere.pt.x + sphere.size/2, sphere.pt.y));
    image_points.push_back(cv::Point2d(sphere.pt.x, sphere.pt.y - sphere.size/2));
    image_points.push_back(cv::Point2d(sphere.pt.x, sphere.pt.y + sphere.size/2));

    std::vector<cv::Point3d> world_points;
    world_points.push_back(cv::Point3d(-0.05f, 0.0f, 0.0f));
    world_points.push_back(cv::Point3d(0.05f, 0.0f, 0.0f));
    world_points.push_back(cv::Point3d(0.0f, -0.05f, 0.0f));
    world_points.push_back(cv::Point3d(0.0f, 0.05f, 0.0f));

    // Zero because we have undistorted the image
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

    // TO DO: should do something about these values as I have them saved in the echo_sounder_camera_handler class
    float instrinsic_data[9] =
      {1066.342505, 0.000000, 940.432119,
       0.000000, 1065.381509, 552.184409,
       0.000000, 0.000000, 1.000000};
    cv::Mat instrinsic = cv::Mat(3, 3, CV_32FC1, instrinsic_data);

    cv::Mat rotation_vector;
    cv::Mat translation_vector;

    cv::solvePnP(world_points, image_points, instrinsic, dist_coeffs, rotation_vector, translation_vector);

    // std::cout << "x: " << translation_vector.at<double>(0, 0) << "\ty: " <<
    //   translation_vector.at<double>(1, 0) << "\tz: " << translation_vector.at<double>(2, 0) <<
    //   "\tdepth: " << distance << "\tconfidence: " << confidence << std::endl;

    cv::Scalar new_edge_point(translation_vector.at<double>(0, 0),
      translation_vector.at<double>(1, 0), translation_vector.at<double>(2, 0));

    this->edge_point_data_.push_back(new_edge_point);
    this->collected_data_points_++;

    // Save data in YAML file for later checks
    // saveData(new_edge_point, distance, confidence);

    return true;
  }

  // Sphere was not detected in image
  this->missed_data_points_++;
  return false;
}


void CalibrationMath::saveData(cv::Scalar edge_point, float distance, int confidence)
{
  // Find file
  std::string filename = "/home/darobot/Research/escalibr/catkin_ws/src/escalibr/output.xml";

  if (0 == this->file_counter)
  {
    // Initializing output file
    TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
    this->out_doc.LinkEndChild(decl);
  }
  else
  {
    // Load document
    this->out_doc.LoadFile(filename);
  }

  this->file_counter++;
  TiXmlElement * save_data_point = new TiXmlElement("data_" + std::to_string(this->file_counter));
  this->out_doc.LinkEndChild(save_data_point);
  save_data_point->SetDoubleAttribute("x", edge_point[0]);
  save_data_point->SetDoubleAttribute("y", edge_point[1]);
  save_data_point->SetDoubleAttribute("z", edge_point[2]);
  save_data_point->SetDoubleAttribute("depth", distance);
  save_data_point->SetDoubleAttribute("confidence", confidence);

  // End file
  this->out_doc.SaveFile(filename);
}


void CalibrationMath::calibrateEchoSounder()
{
  std::cout << std::endl << "TO DO: calibration time..." << std::endl;
}


// TO DO: THE REST DOWN HERE IS NOT USED ... FOR NOW

// int CalibrationMath::checkEllipse(float range)
// {
//   // Check if there are enough data points
//   if (this->ELLIPSE_SAMPLE_SIZE_ > this->edge_point_data_.size())
//   {
//     return -1;
//   }
//
//   // Sort vector by increasing depth measurement
//   std::sort(this->edge_point_data_.begin(), this->edge_point_data_.end(), appEllipse);
//
//   // Check if the data points exists within an acceptable depth range
//   for (size_t i = 0; i + 0 < this->edge_point_data_.size(); i++)
//   {
//     float diff = this->edge_point_data_[i + this->ELLIPSE_SAMPLE_SIZE_ - 1][2] - this->edge_point_data_[i][2];
//     if (diff < range)
//     {
//       // Return pointer to where the set of data points start
//       return i;
//     }
//   }
//
//   return -1;
// }
//
//
// // TO DO: more comments explaining algorithm
// void EllipseMath::fitEllipse(int pt)
// {
//   float TOLERANCE = 0.2;
//   int DIM = 2;
//
//   cv::Mat image = cv::imread("/home/darobot/Research/escalibr/catkin_ws/src/escalibr/collected_data_1/image_15.jpg");
//
//   // A set
//   // Eigen::Vector3f v_a(1423.18, 282.831, 0.898);
//   // Eigen::Vector3f v_b(322.095, 465.748, 0.878);
//   // Eigen::Vector3f v_c(1444.31, 510.07, 0.895);
//   // Eigen::Vector3f v_d(1364.73, 458.061, 0.946);
//   // Eigen::Vector3f v_e(1232.28, 490.754, 0.916);
//   // Eigen::Vector3f v_f(271.483, 541.949, 0.921);
//   // Eigen::Vector3f v_g(1481.85, 578.132, 0.911);
//   // Eigen::Vector3f v_h(1595.21, 511.704, 0.932);
//   // Eigen::Vector3f v_i(328.135, 439.039, 0.906);
//   // Eigen::Vector3f v_j(1402.93, 406.492, 0.923);
//
//   // B set
//   // Eigen::Vector3f v_a(338.647, 516.02, 1.063);
//   // Eigen::Vector3f v_b(448.689, 355.575, 1.067);
//   // Eigen::Vector3f v_c(1426.27, 636.029, 1.088);
//   // Eigen::Vector3f v_d(1395.41, 485.382, 1.109);
//   // Eigen::Vector3f v_e(349.476, 375.162, 1.129);
//   // Eigen::Vector3f v_f(1405.65, 560.766, 1.145);
//   // Eigen::Vector3f v_g(1396.84, 757.2, 1.1);
//   // Eigen::Vector3f v_h(305.817, 752.699, 1.108);
//   // Eigen::Vector3f v_i(1459.34, 498.945, 1.135);
//   // Eigen::Vector3f v_j(1513.0, 555.307, 1.128);
//
//   // C set
//   // Eigen::Vector3f v_a(1395.41, 485.382, 1.109);
//   // Eigen::Vector3f v_b(349.476, 375.162, 1.129);
//   // Eigen::Vector3f v_c(1405.65, 560.766, 1.145);
//   // Eigen::Vector3f v_d(1396.84, 757.2, 1.1);
//   // Eigen::Vector3f v_e(305.817, 752.699, 1.108);
//   // Eigen::Vector3f v_f(1459.34, 498.945, 1.135);
//   // Eigen::Vector3f v_g(1513.0, 555.307, 1.128);
//   // Eigen::Vector3f v_h(1398.1, 627.768, 1.166);
//   // Eigen::Vector3f v_i(1487.51, 516.962, 1.157);
//   // Eigen::Vector3f v_j(401.596, 597.23, 1.177);
//
//   // D set
//   // Eigen::Vector3f v_a(1380.3, 455.133, 1.381);
//   // Eigen::Vector3f v_b(529.75, 321.862, 1.383);
//   // Eigen::Vector3f v_c(1462.01, 387.709, 1.374);
//   // Eigen::Vector3f v_d(1477.98, 602.37, 1.38);
//   // Eigen::Vector3f v_e(1345.26, 534.745, 1.367);
//   // Eigen::Vector3f v_f(566.804, 398.047, 1.414);
//   // Eigen::Vector3f v_g(1576.2, 423.34, 1.403);
//   // Eigen::Vector3f v_h(1427.98, 507.232, 1.419);
//   // Eigen::Vector3f v_i(1390.38, 435.902, 1.429);
//   // Eigen::Vector3f v_j(1642.74, 486.102, 1.448);
//
//   // E set
//   Eigen::Vector3f v_a(566.804, 398.047, 1.414);
//   Eigen::Vector3f v_b(1576.2, 423.34, 1.403);
//   Eigen::Vector3f v_c(1427.98, 507.232, 1.419);
//   Eigen::Vector3f v_d(518.176, 393.92, 1.402);
//   Eigen::Vector3f v_e(1390.38, 435.902, 1.429);
//   Eigen::Vector3f v_f(1642.74, 486.102, 1.448);
//   Eigen::Vector3f v_g(360.091, 369.955, 1.426);
//   Eigen::Vector3f v_h(1423.6, 479.719, 1.456);
//   Eigen::Vector3f v_i(384.514, 472.192, 1.491);
//   Eigen::Vector3f v_j(395.971, 800.199, 1.49);
//
//
//
//   cv::Point c_a(cvRound(v_a[0]), cvRound(v_a[1]));
//   cv::Point c_b(cvRound(v_b[0]), cvRound(v_b[1]));
//   cv::Point c_c(cvRound(v_c[0]), cvRound(v_c[1]));
//   cv::Point c_d(cvRound(v_d[0]), cvRound(v_d[1]));
//   cv::Point c_e(cvRound(v_e[0]), cvRound(v_e[1]));
//   cv::Point c_f(cvRound(v_f[0]), cvRound(v_f[1]));
//   cv::Point c_g(cvRound(v_g[0]), cvRound(v_g[1]));
//   cv::Point c_h(cvRound(v_h[0]), cvRound(v_h[1]));
//   cv::Point c_i(cvRound(v_i[0]), cvRound(v_i[1]));
//   cv::Point c_j(cvRound(v_j[0]), cvRound(v_j[1]));
//   circle(image, c_a, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_b, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_c, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_d, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_e, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_f, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_g, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_h, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_i, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   circle(image, c_j, 5, cv::Scalar(0,0,255), 5, 8, 0 );
//   // cv::imshow("all data", image);
//   // cv::waitKey(0);
//
//
//
//   Eigen::MatrixXf data_points(DIM, this->ELLIPSE_SAMPLE_SIZE_);
//   // for (size_t i = 0; i < this->ELLIPSE_SAMPLE_SIZE_; i++)
//   // {
//   //   std::cout << this->edge_point_data_[pt+1] << std::endl;
//   //   data_points.col(i) = this->edge_point_data_[pt + i].head(2);
//   // }
//
//   // Testing
//   data_points.col(0) = v_a.head(2);
//   data_points.col(1) = v_b.head(2);
//   data_points.col(2) = v_c.head(2);
//   data_points.col(3) = v_d.head(2);
//   data_points.col(4) = v_e.head(2);
//   data_points.col(5) = v_f.head(2);
//   data_points.col(6) = v_g.head(2);
//   data_points.col(7) = v_h.head(2);
//   data_points.col(8) = v_i.head(2);
//   data_points.col(9) = v_j.head(2);
//
//   std::cout << data_points << std::endl;
//
//   Eigen::MatrixXf q = data_points;
//   q.conservativeResize(data_points.rows() + 1, data_points.cols());
//
//   for (size_t i = 0; i < q.cols(); i++)
//   {
//     q(q.rows() - 1, i) = 1;
//   }
//
//   int count = 1;
//   double err = 1;
//
//   const double init_u = 1.0 / static_cast<double>(this->ELLIPSE_SAMPLE_SIZE_);
//   Eigen::MatrixXf u = Eigen::MatrixXf::Constant(this->ELLIPSE_SAMPLE_SIZE_, 1, init_u);
//
//   while (err > TOLERANCE)
//   {
//     // Eigen::MatrixXf Q_tr = q.transpose();
//     Eigen::MatrixXf X = q * u.asDiagonal() * q.transpose();
//     Eigen::MatrixXf M = (q.transpose() * X.inverse() * q).diagonal();
//
//     int j_x, j_y;
//     double maximum = M.maxCoeff(&j_x, &j_y);
//     double step_size = (maximum - DIM - 1) / ((DIM + 1) * (maximum + 1));
//
//     Eigen::MatrixXf new_u = (1 - step_size) * u;
//     new_u(j_x, 0) += step_size;
//
//     // Find error
//     Eigen::MatrixXf u_diff = new_u - u;
//     for (size_t i = 0; i < u_diff.rows(); i++)
//     {
//       for (size_t j = 0; j < u_diff.cols(); j++)
//       {
//         u_diff(i, j) *= u_diff(i, j);  // Square each element of the matrix
//       }
//       err = sqrt(u_diff.sum());
//       count++;
//       u = new_u;
//     }
//   }
//
//   float avg_depth = 0.0;
//   // for (size_t i = pt; i < pt + this->ELLIPSE_SAMPLE_SIZE_; i++)
//   // {
//   //   avg_depth += this->edge_point_data_[i][2];
//   // }
//   avg_depth = v_a[2] + v_b[2] + v_c[2] + v_d[2] + v_e[2] + v_f[2] + v_g[2] + v_h[2] + v_i[2] + v_j[2];
//   avg_depth /= this->ELLIPSE_SAMPLE_SIZE_;
//   std::cout << "average depth: " << avg_depth << std::endl;
//
//   // saveData();
//
//   Eigen::MatrixXf center = data_points * u;
//   std::cout << "center: " << center.transpose() << std::endl;
//
//   float avg_dist_to_center = 0.0;
//   avg_dist_to_center = sin(75 * M_PI / 180) * avg_depth;
//   std::cout << "average distance to center: " << avg_dist_to_center << std::endl;
//
//   Eigen::Vector3f new_ellipse_data {center(0, 0), center(1, 0), avg_dist_to_center};
//   this->ellipse_data_.push_back(new_ellipse_data);
// }
//
//
// std::vector<float> EllipseMath::getEchoSounderTf()
// {
//   Eigen::Vector3f new_ellipse_data {892.019, 581.909, 1.06947};
//   this->ellipse_data_.push_back(new_ellipse_data);
//   Eigen::Vector3f new_ellipse_data_2 {989.36, 541.122, 1.1354};
//   this->ellipse_data_.push_back(new_ellipse_data_2);
//
//   float depth_1 = this->ellipse_data_[0][2];
//   float depth_2 = this->ellipse_data_[1][2];
//
//   float z_1 = 0.0;
//   // Subtract data points and normalize
//   Eigen::Vector3f unit;
//   if (this->ellipse_data_[0][2] < this->ellipse_data_[1][2])
//   {
//     float z_2_d = pow(10000 * (this->ellipse_data_[1][2] - this->ellipse_data_[0][2]), 2);
//     float z_2_x = pow(this->ellipse_data_[1][0] - this->ellipse_data_[0][0], 2);
//     float z_2_y = pow(this->ellipse_data_[1][1] - this->ellipse_data_[0][1], 2);
//     float z_2 = sqrt(z_2_d - z_2_x - z_2_y);
//     std::cout << z_2_d << "\t" << z_2_x << "\t" << z_2_y << std::endl;
//     std::cout << "new z for point 2: " << z_2 << std::endl;
//     this->ellipse_data_[0][2] = z_1;
//     this->ellipse_data_[1][2] = z_2;
//
//     float dist = sqrt(pow(this->ellipse_data_[1][0] - this->ellipse_data_[0][0], 2) +
          // pow(this->ellipse_data_[1][1] - this->ellipse_data_[0][1], 2) + pow(this->ellipse_data_[1][2] -
          // this->ellipse_data_[0][2], 2));
//     std::cout << "distance between two points: " << dist / 10000 << std::endl;
//
//     unit = (this->ellipse_data_[1] - this->ellipse_data_[0]).normalized();
//     std::cout << "unit: " << unit << std::endl;
//     // unit = this->ellipse_data_[1] - this->ellipse_data_[0];
//     // std::cout << "unit: " << unit << std::endl;
//   }
//   else
//   {
//     float z_2 = sqrt(pow(this->ellipse_data_[0][2] - this->ellipse_data_[1][2], 2) -
//      pow(this->ellipse_data_[0][0] - this->ellipse_data_[1][0], 2) - pow(this->ellipse_data_[0][1] -
//      this->ellipse_data_[1][1], 2));
//     this->ellipse_data_[0][2] = z_2;
//     this->ellipse_data_[1][2] = z_1;
//     // unit = (this->ellipse_data_[1] - this->ellipse_data_[0]).normalized();
//     // unit = this->ellipse_data_[1] - this->ellipse_data_[0];
//   }
//
//   // std::cout << "unit: " << unit << std::endl;
//   // std::cout << "unit: " << depth_2 * unit << std::endl;
//
//   // X, Y, Z
//   Eigen::Vector3f pose = this->ellipse_data_[0] - 10000 * depth_1 * unit;
//
//   // Yaw
//   float yaw = atan2(pose[0], -pose[1]);
//
//   // Pitch
//   float pitch = atan2(sqrt(pow(pose[0], 2) + pow(pose[1], 2)), pose[2]);
//
//   std::vector<float> final_parameters = {pose[0], pose[1], pose[2], pitch, yaw};
//   std::cout << "x: " << pose[0] << " y: " << pose[1] << " z: " << pose[2] <<
//      " pitch: " << pitch << " yaw: " << yaw << std::endl;
//   return final_parameters;
// }

}  // namespace escalibr
