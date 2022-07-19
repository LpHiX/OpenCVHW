#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

/// Importing images

void main() {

	VideoCapture cap(0);
	Mat img;

	while (true) {
		cap.read(img);

		cvtColor(img, imgGray, COLOR_BGR)

		imshow("Testing image", img);
		waitKey(1);
	}
}