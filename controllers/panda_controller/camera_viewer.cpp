// camera_viewer.cpp
#include <webots/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace webots;
using namespace cv;

class CameraViewer {
public:
  CameraViewer(Robot* robot, int timeStep) {
    camera = robot->getCamera("camera");
    if (!camera) {
      std::cerr << "Failed to get camera device 'camera'" << std::endl;
      throw std::runtime_error("Camera init failed");
    }
    camera->enable(timeStep);
    width = camera->getWidth();
    height = camera->getHeight();
    std::cout << "[INFO] Camera enabled: " << width << "x" << height << std::endl;
  }

  void process() {
    const unsigned char *image = camera->getImage();
    if (!image) return;

    Mat bgr(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        bgr.at<Vec3b>(y, x)[0] = camera->imageGetBlue(image, width, x, y);
        bgr.at<Vec3b>(y, x)[1] = camera->imageGetGreen(image, width, x, y);
        bgr.at<Vec3b>(y, x)[2] = camera->imageGetRed(image, width, x, y);
      }
    }

    imshow("Webots Camera", bgr);
    waitKey(1);
  }

private:
  Camera* camera;
  int width, height;
};
