#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "cameralibrary.h"
#include <WS2tcpip.h>
#include <Windows.h>
#pragma comment(lib, "ws2_32.lib")

using namespace CameraLibrary;
using namespace std;
using namespace cv;

string ipAddress = "127.0.0.1";
int port = 54000;
bool online = true;
bool live_camera = true;

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

Vector3 pos_raw, pos_off, pos_out;
Vector3 pos_zero, pos_z;
Vector3 pos_scale(1.28399, -1.14709, -5.51937), pos_offset(258.424, 1130.43, 1105.56);
//Vector3 pos_scale(1.399962, -1.432813, -8.621343), pos_offset(-20.066824, 951.331665, 931.941101);
//Vector3 pos_scale(0.844572, -0.865757, -1.595348), pos_offset(-93.856209, 2048.592285, 2887.883301);

float cam_angle;

int meter_radius_value;
const int meter_radius_max = 100;
int lum_thresh_value;
const int lum_thresh_max = 255;
int calibration_value;
const int calibration_max = 3000;
int crop_value;
const int crop_max = 1000;

Point2f center;
float radius;

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
			cout << "pos_scale(" << pos_scale.x << ", " << pos_scale.y << ", " << pos_scale.z << "), pos_offset("<< pos_offset.x << ", " << pos_offset.y <<", " << pos_offset.z << ");" << endl;
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
	cout << "Main Program Starting" << endl;

	CameraLibrary_EnableDevelopment();
	CameraManager::X().WaitForInitialization();

	Camera* camera = CameraManager::X().GetCamera();
	if (!camera)
	{
		cout << "Camera is invalid, using debug video" << endl;
		live_camera = false;
	}

	const int cameraWidth = 2048;
	const int cameraHeight = 1088;

	string path = "Resources/test_video.mkv";
	VideoCapture cap(path);

	if (live_camera)
	{
		camera->SetVideoType(Core::MJPEGMode);
		camera->SetExposure(50);
		//camera->SetThreshold(0);
		camera->SetThreshold(200);
		camera->SetIntensity(15);
		camera->SetFrameRate(120);
		cout << camera->FrameRate() << endl;
		cout << camera->MaximumFrameRateValue() << endl;
		cout << camera->MinimumIntensity() << endl;
		cout << camera->MinimumExposureValue() << endl;
		cout << camera->MinimumThreshold() << endl;
		
		//camera->SetTextOverlay(true);

		camera->Start();
		cout << "Camera initialized, Starting loop" << endl;

	}

	char imageBuffer[cameraWidth * cameraHeight];


	Mat1b img_raw;
	Mat1b img;
	Mat3b img_out;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	// Control panel setup

	Mat3b controlpanel_img(160, 300, Vec3b(100, 255, 0));

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
	calibration_value = 1500;
	createTrackbar("Calibration Value", "Control Panel", &calibration_value, calibration_max);
	crop_value = 400;
	createTrackbar("Crop value", "Control Panel", &crop_value, crop_max);

	// Initializing WinSock

	WSAData data;
	WORD ver = MAKEWORD(2, 2);
	int wsResult = WSAStartup(ver, &data);
	if (wsResult != 0)
	{
		cerr << "Can't start Winsock, Err #" << wsResult << endl;
		return 0;
	}

	// Create Socket
	SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == INVALID_SOCKET)
	{
		cerr << "Can't create socket, Err #" << WSAGetLastError() << endl;
		WSACleanup();
		return 0;
	}

	// Fill in a hint structure
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(port);
	inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

	// uhh

	int connResult = connect(sock, (sockaddr*)&hint, sizeof(hint));
	if (connResult == SOCKET_ERROR)
	{
		cerr << "Can't connect to server, Err #" << WSAGetLastError() << " Starting offline mode" << endl;
		online = false;
		closesocket(sock);
		WSACleanup();
	}
	
	// Do-while loop to send and recieve data
	char buf[4096];
	string pos_string;
	int64 tick;

	while (true)
	{
		tick = getTickCount();
		if (live_camera)
		{
			Frame* frame = camera->GetFrame();
			if (!frame)
			{
				continue;
			}

			frame->Rasterize(cameraWidth, cameraHeight, cameraWidth, 8, imageBuffer);
			img = Mat(cameraHeight, cameraWidth, CV_8UC1, &imageBuffer);
			frame->Release();

			Rect crop_rect(0, crop_value, round(cameraWidth), round(cameraHeight) - crop_value);
			img = img(crop_rect);
		}
		else
		{
			Mat3b raw_img;
			cap.read(raw_img);
			if (raw_img.empty())
			{
				continue;
			}
			cvtColor(raw_img, img, COLOR_BGR2GRAY);
		}
		inRange(img, lum_thresh_value, 255, img);
		findContours(img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		size_t max_int = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > contourArea(contours[max_int]))
			{
				max_int = i;
			}
		}
		//cvtColor(img, img_out, COLOR_GRAY2BGR);

		if (contours.size() > 0)
		{
			minEnclosingCircle(contours[max_int], center, radius);

			//drawContours(img, contours, max_int, Scalar(255, 0, 0), 2, LINE_8, hierarchy, 0);

			//putText(img, to_string(radius), Point2f(10, 100), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, LINE_8, false);

			Point2f trans_center = center - Point2f(img.size().width / 2, img.size().height / 2);
			Vector3 screen_pos = Vector3(trans_center.x, trans_center.y, 200);

			pos_raw = screen_pos * ((float)meter_radius_value / radius);

			//Vector3 pos_tran = Vector3(pos_raw).transform(cam_angle);

			pos_off = Vector3(pos_raw) - Vector3(pos_offset);//.transform(cam_angle);
			pos_out = Vector3(pos_off) * pos_scale;

			pos_out = pos_out.x * pos_out.x + pos_out.y * pos_out.y + pos_out.z * pos_out.z < 100000000 ? pos_out : Vector3(0, 0, 0);
		}
		/*
		int divider = 3;
		resize(img, img, Size(round(img.cols / divider), round(img.rows / divider)), INTER_NEAREST);
		resize(img_raw, img_raw, Size(round(img_raw.cols / divider), round(img_raw.rows / divider)), INTER_NEAREST);
		cvtColor(img_raw, img_out, COLOR_GRAY2RGB);
		
		putText(img_out, to_string(pos_out.x), Point2f(10, 150), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 255, 0), 1, LINE_8, false);
		putText(img_out, to_string(pos_out.y), Point2f(10, 170), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 255, 0), 1, LINE_8, false);
		putText(img_out, to_string(pos_out.z), Point2f(10, 190), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 255, 0), 1, LINE_8, false);
		circle(img_out, center / divider, radius / divider, Scalar(0, 255, 0), 1);

		imshow("Input", img_out);*/
		imshow("Output", img);

		if (online)
		{
			pos_string = to_string(pos_out.x) + " " + to_string(pos_out.y) + " " + to_string(pos_out.z) + " ";
			int sendResult = send(sock, pos_string.c_str(), pos_string.size() + 1, 0);
			if (sendResult != SOCKET_ERROR)
			{
				// Wait for response
				ZeroMemory(buf, 4096);
				int bytesRecieved = recv(sock, buf, 4096, 0);
				if (bytesRecieved > 0)
				{
					// Echo response to 
					//cout << "SERVER> " << string(buf, 0, bytesRecieved) << endl;
				}
			}
		}
		char key = (char)cv::waitKey(1);
		//cout << (getTickCount() - tick) / getTickFrequency()<< endl;
		if (key == 27) break;
	}
	cout << "Closing down" << endl;
	if (live_camera)
	{
		closesocket(sock);
		WSACleanup();
	}
	camera->Release();
	CameraManager::X().Shutdown();
	return 0;
}