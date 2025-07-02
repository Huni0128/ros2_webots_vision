#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#define TIME_STEP 32

using namespace webots;

int main() {
  Robot robot;

  Camera *camera = robot.getCamera("camera");
  if (!camera) {
    std::cerr << "Failed to get camera device 'camera'" << std::endl;
    return 1;
  }

  camera->enable(TIME_STEP);
  int width = camera->getWidth();
  int height = camera->getHeight();

  std::cout << "[INFO] Camera enabled: " << width << "x" << height << std::endl;

  while (robot.step(TIME_STEP) != -1) {
    const unsigned char *image = camera->getImage();
    if (!image)
      continue;

    // Webots 이미지를 OpenCV BGR 이미지로 변환
    cv::Mat bgr(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        bgr.at<cv::Vec3b>(y, x)[0] = camera->imageGetBlue(image, width, x, y);
        bgr.at<cv::Vec3b>(y, x)[1] = camera->imageGetGreen(image, width, x, y);
        bgr.at<cv::Vec3b>(y, x)[2] = camera->imageGetRed(image, width, x, y);
      }
    }

    cv::imshow("Webots Camera", bgr);
    if (cv::waitKey(1) == 27)  // ESC to quit
      break;
  }

  return 0;
}
