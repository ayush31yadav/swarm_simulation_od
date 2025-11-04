#include <opencv2/opencv.hpp>

int main() {
    int width = 512, height = 256;
    cv::Mat gradient(height, width, CV_8UC1);

    for (int x = 0; x < width; ++x) {
        gradient.col(x).setTo(uchar(x * 255 / (width - 1)));
    }

    cv::imshow("Gradient", gradient);
    cv::waitKey(0);
    return 0;
}