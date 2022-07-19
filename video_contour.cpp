#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "cameralibrary.h"
using namespace CameraLibrary;
using namespace std;
using namespace cv;

int meter_radius_value;
const int meter_radius_max = 100;

int lum_thresh_value;
const int lum_thresh_max = 255;

float x_scale, y_scale, z_scale, x_offset, y_offset, z_offset;

struct Vector3 {
	float x, y, z;
	Vector3() {
		x, y, z = 0;
	}
	Vector3(const float& x, const float& y, const float& z) :
		x(x), y(y), z(z) {}
	Vector3& operator =(const Vector3& a) {
		x = a.x;
		y = a.y;
		z = a.z;
		return *this;
	}
	Vector3& operator + (const Vector3& a) {
		x = x + a.x;
		y = y + a.y;
		z = z + a.z;
		return *this;
	}
	Vector3& operator - (const Vector3& a) {
		x = x - a.x;
		y = y - a.y;
		z = z - a.z;
		return *this;
	}
	Vector3& operator * (const float& a) {
		x = x * a;
		y = y * a;
		z = z * a;
		return *this;
	}
};

/// Importing images
void nothing() {
}

Mat3b canvas;
Rect button;
Rect button_calzero;
Rect button_calx;
Rect Button_caly;
Rect Button_calz;

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		if (button.contains(Point(x, y)))
		{
			cout << "Clicked!" << endl;
			rectangle(canvas(button), button, Scalar(0, 0, 255), 2);
		}
	}
	if (event == EVENT_LBUTTONUP)
	{
		rectangle(canvas, button, Scalar(200, 200, 200), 2);
	}

	imshow("Control Panel", canvas);
}
int main() {

	string path = "Resources/test_video.mkv";
	VideoCapture cap(0);

	Mat img;
	Mat img_hsv;
	Mat img_mask;
	Mat img_blur;

	int thresh = 100;
	Mat canny_output;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	RNG rng(12345);

	Vector3 position;

	// Control panel setup
	
	Mat3b controlpanel_img(300, 300, Vec3b(100, 255, 0));
	button = Rect(0, 0, controlpanel_img.cols / 2, 50);
	canvas = Mat3b(controlpanel_img.rows + button.height, controlpanel_img.cols, Vec3b(0, 0, 0));
	canvas(button) = Vec3b(200, 200, 200);
	putText(canvas(button), "Click me", Point(button.width * 0.35, button.height * 0.7), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0));
	controlpanel_img.copyTo(canvas(Rect(0, button.height, controlpanel_img.cols, controlpanel_img.rows)));


	namedWindow("Control Panel", WINDOW_AUTOSIZE);
	setMouseCallback("Control Panel", callBackFunc);

	imshow("Control Panel", canvas);

	/// Calibration, Startsize

	meter_radius_value = 60;
	createTrackbar("1mRadius", "Control Panel", &meter_radius_value, meter_radius_max);
	lum_thresh_value = 100;
	createTrackbar("LumThresh", "Control Panel", &lum_thresh_value, lum_thresh_max);


	while (true) {
		cap.read(img);
		if (img.empty()) {
			break;
		}
		cvtColor(img, img_hsv, COLOR_BGR2HSV);
		inRange(img_hsv, Scalar(0, 0, lum_thresh_value), Scalar(180, 255, 255), img_mask);
		imshow("Contour Input", img_mask);


		findContours(img_mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		//Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
		size_t max_int = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > contourArea(contours[max_int])) {
				max_int = i;
			}
			//Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		}
		//drawContours(img, contours, max_int, Scalar(255, 0, 0), 4, LINE_8, hierarchy, 0);

		if (contours.size() > 0) {
			Point2f center;
			float radius;

			minEnclosingCircle(contours[max_int], center, radius);
			circle(img, center, radius, Scalar(0, 255, 0), 4);

			drawContours(img, contours, max_int, Scalar(255, 0, 0), 2, LINE_8, hierarchy, 0);

			putText(img, to_string(radius), Point2f(100, 100), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);

			Point2f trans_center = center - Point2f(img.size().width / 2, img.size().height / 2);
			Vector3 screen_pos = Vector3(trans_center.x, trans_center.y, 1);

			Vector3 pos = screen_pos * ((float) meter_radius_value / radius);
			putText(img, to_string(pos.z), Point2f(100, 190), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
			putText(img, to_string(pos.x), Point2f(100, 150), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
			putText(img, to_string(pos.y), Point2f(100, 170), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
		}
		imshow("Output", img);
		waitKey(1);
	}
	return 0;
}