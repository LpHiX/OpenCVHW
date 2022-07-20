#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "cameralibrary.h"

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
using namespace CameraLibrary;
using namespace std;
using namespace cv;

struct Vector3
{
	float x, y, z;
	Vector3()
	{
		x, y, z = 0;
	}
	Vector3(const float& x, const float& y, const float& z) :
		x(x), y(y), z(z)
	{}
	Vector3& operator =(const Vector3& a)
	{
		x = a.x;
		y = a.y;
		z = a.z;
		return *this;
	}
	Vector3& operator + (const Vector3& a)
	{
		x = x + a.x;
		y = y + a.y;
		z = z + a.z;
		return *this;
	}
	Vector3& operator - (const Vector3& a)
	{
		x = x - a.x;
		y = y - a.y;
		z = z - a.z;
		return *this;
	}
	Vector3& operator * (const float& a)
	{
		x = x * a;
		y = y * a;
		z = z * a;
		return *this;
	}
	Vector3& operator * (const Vector3& a)
	{
		x = x * a.x;
		y = y * a.y;
		z = z * a.z;
		return *this;
	}
	/*Vector3 transform(const float& angle)
	{
		float y0 = y, z0 = z;
		z = z0 * cosf(angle) - y0 * sinf(angle);
		y = z0 * sinf(angle) + y0 * cosf(angle);
		return *this;
	}
	Vector3 flip_depth()
	{
		y = -y;
		return *this;
	}*/
};

Vector3 pos_raw, pos_off, pos_offset, pos_out;
Vector3 pos_zero, pos_z;
Vector3 pos_scale( 0.329830, -0.341303, -0.788949);

float cam_angle;

int meter_radius_value;
const int meter_radius_max = 100;
int lum_thresh_value;
const int lum_thresh_max = 255;
int calibration_value;
const int calibration_max = 1000;



/// Importing images
void nothing()
{}

Mat3b canvas;
Rect button;
Rect button_calzero;
Rect button_calx;
Rect button_caly;
Rect button_calz;

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		if (button.contains(Point(x, y)))
		{
			rectangle(canvas, button, Scalar(0, 0, 255), 2);
			cout << "Cam Angle" + to_string(cam_angle) << endl;
			cout << "Scale X" + to_string(pos_scale.x) << endl;
			cout << "Scale Y" + to_string(pos_scale.y) << endl;
			cout << "Scale Z" + to_string(pos_scale.z) << endl;
		}
		else if (button_calzero.contains(Point(x, y)))
		{
			rectangle(canvas, button_calzero, Scalar(0, 0, 255), 2);
			pos_offset = pos_raw;
			pos_zero = pos_raw;
		}
		else if (button_calx.contains(Point(x, y)))
		{
			rectangle(canvas, button_calx, Scalar(0, 0, 255), 2);
			pos_scale.x = calibration_value / pos_off.x;
		}
		else if (button_caly.contains(Point(x, y)))
		{
			rectangle(canvas, button_caly, Scalar(0, 0, 255), 2);
			pos_scale.y = calibration_value / pos_off.y;
		}
		else if (button_calz.contains(Point(x, y)))
		{
			rectangle(canvas, button_calz, Scalar(0, 0, 255), 2);
			pos_scale.z = calibration_value / pos_off.z;
			pos_z = pos_raw;

			/*Vector3 floor_diff = pos_z - pos_zero;
			cout << to_string(floor_diff.x) + " Y " + to_string(floor_diff.y) + " Z " + to_string(floor_diff.z) << endl;
			cam_angle = acosf(abs(floor_diff.z) / sqrtf(floor_diff.y * floor_diff.y + floor_diff.z * floor_diff.z));
			cout << abs(floor_diff.z) / sqrtf(floor_diff.y * floor_diff.y + floor_diff.z * floor_diff.z) << endl;*/

		}
	}
	if (event == EVENT_LBUTTONUP)
	{
		rectangle(canvas, button, Scalar(200, 200, 200), 2);
		rectangle(canvas, button_calzero, Scalar(200, 200, 200), 2);
		rectangle(canvas, button_calx, Scalar(200, 200, 200), 2);
		rectangle(canvas, button_caly, Scalar(200, 200, 200), 2);
		rectangle(canvas, button_calz, Scalar(200, 200, 200), 2);
	}

	imshow("Control Panel", canvas);
}
int main()
{

	string path = "Resources/test_video.mkv";
	VideoCapture cap(path);

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
	Mat normalized_img;

	button = Rect(controlpanel_img.cols / 2, 0, controlpanel_img.cols / 2, 50);
	button_calzero = Rect(controlpanel_img.cols / 2, 50, controlpanel_img.cols / 2, 50);
	button_calx = Rect(0, 0, controlpanel_img.cols / 2, 50);
	button_caly = Rect(0, 50, controlpanel_img.cols / 2, 50);
	button_calz = Rect(0, 100, controlpanel_img.cols / 2, 50);

	canvas = Mat3b(controlpanel_img.rows, controlpanel_img.cols, Vec3b(0, 0, 0));
	controlpanel_img.copyTo(canvas(Rect(0, 0, controlpanel_img.cols, controlpanel_img.rows)));
	canvas(button) = Vec3b(200, 200, 200);
	canvas(button_calzero) = Vec3b(200, 255, 10);
	canvas(button_calx) = Vec3b(100, 100, 200);
	canvas(button_caly) = Vec3b(100, 200, 000);
	canvas(button_calz) = Vec3b(200, 100, 000);

	putText(canvas(button), "Debug", Point(button.width * 0.20, button.height * 0.7), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0));
	putText(canvas(button_calzero), "Cal 0", Point(button.width * 0.20, button.height * 0.7), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0));
	putText(canvas(button_calx), "Cal X", Point(button.width * 0.20, button.height * 0.7), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0));
	putText(canvas(button_caly), "Cal Y", Point(button.width * 0.20, button.height * 0.7), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0));
	putText(canvas(button_calz), "Cal Z", Point(button.width * 0.20, button.height * 0.7), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0));
	rectangle(canvas, button, Scalar(200, 200, 200), 2);
	rectangle(canvas, button_calzero, Scalar(200, 200, 200), 2);
	rectangle(canvas, button_calx, Scalar(200, 200, 200), 2);
	rectangle(canvas, button_caly, Scalar(200, 200, 200), 2);
	rectangle(canvas, button_calz, Scalar(200, 200, 200), 2);

	namedWindow("Control Panel", WINDOW_AUTOSIZE);
	setMouseCallback("Control Panel", callBackFunc);

	imshow("Control Panel", canvas);

	/// Calibration, Startsize

	meter_radius_value = 60;
	createTrackbar("1mRadius", "Control Panel", &meter_radius_value, meter_radius_max);
	lum_thresh_value = 30;
	createTrackbar("LumThresh", "Control Panel", &lum_thresh_value, lum_thresh_max);
	calibration_value = 500;
	createTrackbar("Calibration Value", "Control Panel", &calibration_value, calibration_max);

	UdpTransmitSocket transmitSocket(IpEndpointName("127.0.0.1", 7000));
	/*
	char buffer[OUTPUT_BUFFER_SIZE];
	osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

	p << osc::BeginBundleImmediate
		<< osc::BeginMessage("/test1")
		<< true << 23 << (float)3.1415 << "hello" << osc::EndMessage
		<< osc::BeginMessage("/test2")
		<< true << 24 << (float)10.8 << "world" << osc::EndMessage
		<< osc::EndBundle;

	transmitSocket.Send(p.Data(), p.Size());*/

	while (true)
	{
		cap.read(img);
		if (img.empty())
		{
			waitKey(0);
			break;
		}
		cvtColor(img, img_hsv, COLOR_BGR2HSV);

		//img.convertTo(normalized_img, -1, 20, 0);
		imshow("Original", img);

		inRange(img_hsv, Scalar(0, 0, lum_thresh_value), Scalar(180, 255, 255), img_mask);
		imshow("Contour Input", img_mask);


		findContours(img_mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		//Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
		size_t max_int = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > contourArea(contours[max_int]))
			{
				max_int = i;
			}
			//Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		}
		//drawContours(img, contours, max_int, Scalar(255, 0, 0), 4, LINE_8, hierarchy, 0);

		if (contours.size() > 0)
		{
			Point2f center;
			float radius;

			minEnclosingCircle(contours[max_int], center, radius);
			circle(img, center, radius, Scalar(0, 255, 0), 4);

			drawContours(img, contours, max_int, Scalar(255, 0, 0), 2, LINE_8, hierarchy, 0);

			putText(img, to_string(radius), Point2f(100, 100), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);

			Point2f trans_center = center - Point2f(img.size().width / 2, img.size().height / 2);
			Vector3 screen_pos = Vector3(trans_center.x, trans_center.y, 200);

			pos_raw = screen_pos * ((float)meter_radius_value / radius);

			//Vector3 pos_tran = Vector3(pos_raw).transform(cam_angle);

			pos_off = Vector3(pos_raw) - Vector3(pos_offset);//.transform(cam_angle);
			pos_out = Vector3(pos_off) * pos_scale;


			putText(img, to_string(pos_out.x), Point2f(100, 150), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
			putText(img, to_string(pos_out.y), Point2f(100, 170), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
			putText(img, to_string(pos_out.z), Point2f(100, 190), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
			putText(img, to_string(pos_raw.x), Point2f(100, 250), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
			putText(img, to_string(pos_raw.y), Point2f(100, 270), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
			putText(img, to_string(pos_raw.z), Point2f(100, 290), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);
		}
		imshow("Output", img);

		char key = (char)cv::waitKey(1);
		if (key == 27) break;
	}
	return 0;
}