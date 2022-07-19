#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "cameralibrary.h"
using namespace CameraLibrary;
using namespace std;
using namespace cv;

/// Importing images

int main() {

	VideoCapture cap(0);
	Mat img;
	Mat img_gray;
	Mat img_blur;

	int thresh = 100;
	Mat canny_output;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	RNG rng(12345);

	for (int i = 0; i < 1000; i++) {
		cap.read(img);
		cvtColor(img, img_gray, COLOR_BGR2GRAY);
		blur(img_gray, img_blur, Size(5, 5));

		imshow("Blurred", img_blur);

		Canny(img_blur, canny_output, thresh, thresh * 2);
		findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
		for (size_t i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
		}

		imshow("Testing image", drawing);
		waitKey(10);
	}
	return 0;
}