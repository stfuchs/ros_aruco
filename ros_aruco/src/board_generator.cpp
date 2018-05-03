#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

int main(int argc, char** argv)
{
  int squares_x = 9;
  int squares_y = 13;
  float square_length = 0.065;
  float marker_length = 0.04;

  float inch_to_pixel = 72;
  float meter_to_inch = 1./0.0254;

  auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  auto board = cv::aruco::CharucoBoard::create(
    squares_x, squares_y, square_length, marker_length, dict);
  cv::Mat img;
  board->draw(
    cv::Size(
      squares_x * square_length * meter_to_inch * inch_to_pixel,
      squares_y * square_length * meter_to_inch * inch_to_pixel),
    img, 0);
  cv::imwrite("board.png", img);

  return 0;
}
