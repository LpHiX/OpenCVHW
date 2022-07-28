#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "cameralibrary.h"
#include <WS2tcpip.h>
#include <Windows.h>
#include "tinyosc.h"
#pragma comment(lib, "ws2_32.lib")

using namespace CameraLibrary;

struct Vector3
{
	float x, y, z;
	Vector3()
	{
		x, y, z = 0;
	}
	Vector3(const float& x, const float& y, const float& z)
		: x(x), y(y), z(z)
	{}
	Vector3 Add(const Vector3& other) const
	{
		return Vector3(x + other.x, y + other.y, z + other.z);
	}
	Vector3 operator + (const Vector3& other) const
	{
		return Add(other);
	}
	Vector3 Subtract(const Vector3& other) const
	{
		return Vector3(x - other.x, y - other.y, z - other.z);
	}
	Vector3 operator- (const Vector3& other) const
	{
		return Subtract(other);
	}
	Vector3 Multiply(const float& a) const
	{
		return Vector3(x * a, y * a, z * a);
	}
	Vector3 Multiply(const Vector3& other) const
	{
		return Vector3(x * other.x, y * other.y, z * other.z);
	}
	Vector3 operator * (const float& a) const
	{
		return Multiply(a);
	}
	Vector3 operator * (const Vector3& other) const
	{
		return Multiply(other);
	}
	float MagnitudeSquared() const
	{
		return x * x + y * y + z * z;
	}
	float Magnitude() const
	{
		return sqrtf(MagnitudeSquared());
	}
	Vector3 RotateX(float radians) const
	{
		return Vector3(x, y * cosf(radians) - z * sinf(radians), y * sinf(radians) + z * cosf(radians));
	}
};

std::ostream& operator<<(std::ostream& stream, const Vector3& other)
{
	stream << other.x << ", " << other.y << ", " << other.z;
	return stream;
}

Vector3 pos_raw, pos_centered, pos_rotated, pos_out;
Vector3 pos_offset(313.33, 321.968, 2090.76), pos_scale(0.814982, 0.824074, 0.178267);
Vector3 rawpos_0(313.33, 321.968, 2090.76), rawpos_x(436.032, 320.33, 2090.76), rawpos_y(313.338, 201.041, 2100.85), rawpos_z(316.463, 324.976, 1529.82);
float x_angle = 3.13623;


int lum_thresh_value = 30;
const int lum_thresh_max = 255;
int cal_value = 100;
const int cal_max = 3000;
int crop_value = 0;
const int crop_max = 1000;

cv::Point2f center;
float radius;

cv::Mat3b canvas(180, 300, cv::Vec3b(0, 255, 0));
cv::Rect cal_x(0, 0, 150, 60);
cv::Rect cal_y(0, 60, 150, 60);
cv::Rect cal_z(0, 120, 150, 60);
cv::Rect cal_0(150, 60, 150, 60);
cv::Rect cal_debug(150, 0, 150, 60);

bool online = true;
bool use_camera = true;
std::string path = "Resources/test_video.mkv";
cv::VideoCapture cap(path);
cv::Mat img_raw;
cv::Mat1b img;
cv::Mat3b img_out;
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;
const int cameraWidth = 2048;
const int cameraHeight = 1088;
char imageBuffer[cameraWidth * cameraHeight];

static void on_lumthresh(int value, void*) { lum_thresh_value = value; }
static void on_calvalue(int value, void*) { cal_value = value; }
static void on_cropvalue(int value, void*) { crop_value = value; }
void recal_xaxis()
{
	x_angle = atan2f((rawpos_z - pos_offset).y, (rawpos_z - pos_offset).z);
	pos_scale.y = cal_value / (rawpos_y - pos_offset).Magnitude();
	pos_scale.z = cal_value / (rawpos_z - pos_offset).Magnitude();
}
void mouseClickFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		if (cal_0.contains(cv::Point(x, y)))
		{
			rectangle(canvas, cal_0, cv::Scalar(0, 255, 0), 2);
			rawpos_0 = pos_raw;
			pos_offset = pos_raw;
		}
		if (cal_x.contains(cv::Point(x, y)))
		{
			rectangle(canvas, cal_x, cv::Scalar(0, 255, 0), 2);
			rawpos_x = pos_raw;
			pos_scale.x = cal_value / pos_centered.x;
		}
		if (cal_y.contains(cv::Point(x, y)))
		{
			rectangle(canvas, cal_y, cv::Scalar(0, 255, 0), 2);
			rawpos_y = pos_raw;
			recal_xaxis();
			
		}
		if (cal_z.contains(cv::Point(x, y)))
		{
			rectangle(canvas, cal_z, cv::Scalar(0, 255, 0), 2);
			rawpos_z = pos_raw;
			recal_xaxis();

		}
		if (cal_debug.contains(cv::Point(x, y)))
		{
			rectangle(canvas, cal_debug, cv::Scalar(0, 255, 0), 2);
			std::cout << "Vector3 pos_offset(" << pos_offset << "), pos_scale(" << pos_scale << ");" << std::endl;
			std::cout << "Vector3 rawpos_0(" << rawpos_0 << "), rawpos_x(" << rawpos_x << "), rawpos_y(" << rawpos_y << "), rawpos_z(" << rawpos_z << ");" << std::endl;
			std::cout << "float x_angle = " << x_angle << ";" << std::endl;
		}
	}
	if (event == cv::EVENT_LBUTTONUP)
	{
		rectangle(canvas, cal_0, cv::Scalar(100, 100, 100), 2);
		rectangle(canvas, cal_x, cv::Scalar(100, 100, 100), 2);
		rectangle(canvas, cal_y, cv::Scalar(100, 100, 100), 2);
		rectangle(canvas, cal_z, cv::Scalar(100, 100, 100), 2);
		rectangle(canvas, cal_debug, cv::Scalar(100, 100, 100), 2);
	}
	cv::imshow("Control Panel", canvas);
}
void setupControlPanel()
{
	canvas(cal_x) = cv::Vec3b(100, 100, 200);
	canvas(cal_y) = cv::Vec3b(100, 200, 100);
	canvas(cal_z) = cv::Vec3b(200, 100, 100);
	canvas(cal_0) = cv::Vec3b(200, 200, 100);
	canvas(cal_debug) = cv::Vec3b(100, 100, 100);
	cv::putText(canvas(cal_x), "Cal X", cv::Point(30, 45), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
	cv::putText(canvas(cal_y), "Cal Y", cv::Point(30, 45), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
	cv::putText(canvas(cal_z), "Cal Z", cv::Point(30, 45), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
	cv::putText(canvas(cal_0), "Cal 0", cv::Point(30, 45), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
	cv::putText(canvas(cal_debug), "Debug", cv::Point(30, 45), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
	rectangle(canvas, cal_0, cv::Scalar(100, 100, 100), 2);
	rectangle(canvas, cal_x, cv::Scalar(100, 100, 100), 2);
	rectangle(canvas, cal_y, cv::Scalar(100, 100, 100), 2);
	rectangle(canvas, cal_y, cv::Scalar(100, 100, 100), 2);
	rectangle(canvas, cal_z, cv::Scalar(100, 100, 100), 2);
	cv::imshow("Control Panel", canvas);
	cv::setMouseCallback("Control Panel", mouseClickFunc);

	cv::createTrackbar("Brightness threshold", "Control Panel", nullptr, lum_thresh_max, on_lumthresh);
	cv::createTrackbar("Calibration value", "Control Panel", nullptr, cal_max, on_calvalue);
	cv::createTrackbar("Crop value", "Control Panel", nullptr, crop_max, on_cropvalue);
	cv::setTrackbarPos("Brightness threshold", "Control Panel", lum_thresh_value);
	cv::setTrackbarPos("Calibration value", "Control Panel", cal_value);
	cv::setTrackbarPos("Crop value", "Control Panel", crop_value);
}

int main()
{
	std::cout << "Main Program Starting" << std::endl;

	CameraLibrary_EnableDevelopment();
	CameraManager::X().WaitForInitialization();

	Camera* camera = CameraManager::X().GetCamera();
	if (!camera)
	{
		std::cout << "Camera is invalid, using debug video" << std::endl;
		use_camera = false;
	}

	if (use_camera)
	{
		camera->SetVideoType(Core::MJPEGMode);
		camera->SetExposure(50);
		camera->SetThreshold(200);
		camera->SetIntensity(15);
		camera->SetFrameRate(120);
		camera->Start();
		std::cout << "Camera initialized, Starting loop" << std::endl;
	}

	setupControlPanel();

	WSAData data;
	WORD version = MAKEWORD(2, 2);
	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		std::cout << "Can't start Winsock! " << wsOk;
		return 0;
	}

	sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_port = htons(54000); 
	inet_pton(AF_INET, "127.0.0.1", &server.sin_addr); 
	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);

	std::string pos_string;
	int64 tick;

	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27) break;
		tick = cv::getTickCount();
		if (use_camera)
		{
			Frame* frame = camera->GetFrame();
			if (!frame)
			{
				continue;
			}

			frame->Rasterize(cameraWidth, cameraHeight, cameraWidth, 8, imageBuffer);
			img_raw = cv::Mat(cameraHeight, cameraWidth, CV_8UC1, &imageBuffer);
			frame->Release();
			inRange(img_raw, lum_thresh_value, 255, img);
		}
		else
		{
			cap.read(img_raw);
			if (img_raw.empty())
			{
				continue;
			}
			cvtColor(img_raw, img, cv::COLOR_BGR2GRAY);
			inRange(img, lum_thresh_value, 255, img);
		}
		findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		size_t max_int = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > contourArea(contours[max_int]))
			{
				max_int = i;
			}
		}

		if (contours.size() > 0)
		{
			minEnclosingCircle(contours[max_int], center, radius);
			pos_raw = Vector3(center.x, center.y, 10000 / radius);
			pos_centered = pos_raw - pos_offset;
			pos_rotated = pos_centered.RotateX(-x_angle);
			pos_out = pos_rotated * pos_scale;

			pos_out = pos_out.MagnitudeSquared() < 100000000 ? pos_out : Vector3(0, 0, 0);
		}

		int divider = 1;
		resize(img, img, cv::Size(round(img.cols / divider), round(img.rows / divider)), cv::INTER_NEAREST);
		resize(img_raw, img_raw, cv::Size(round(img_raw.cols / divider), round(img_raw.rows / divider)), cv::INTER_NEAREST);
		if (use_camera)
		{
			cvtColor(img_raw, img_out, cv::COLOR_GRAY2RGB);
		}
		else
		{
			img_out = img_raw;
		}

		putText(img_out, std::to_string(pos_out.x), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 255, 0), 1, cv::LINE_8, false);
		putText(img_out, std::to_string(pos_out.y), cv::Point2f(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 255, 0), 1, cv::LINE_8, false);
		putText(img_out, std::to_string(pos_out.z), cv::Point2f(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 255, 0), 1, cv::LINE_8, false);
		circle(img_out, center / divider, radius / divider, cv::Scalar(0, 255, 0), 1);

		imshow("Input", img);
		imshow("Output", img_out);

		if (online)
		{
			pos_string = "POS_DATA " + std::to_string(pos_out.x) + " " + std::to_string(pos_out.y) + " " + std::to_string(pos_out.z) + " -----------------------------";
			int sendOk = sendto(out, pos_string.c_str(), pos_string.size() + 1, 0, (sockaddr*)&server, sizeof(server));
			if (sendOk == SOCKET_ERROR)
			{
				std::cout << "Error in sending position data! " << WSAGetLastError() << std::endl;
			}
		}
		//cout << (getTickCount() - tick) / getTickFrequency()<< endl;
	}
	std::cout << "Closing down" << std::endl;
	if (use_camera)
	{
		camera->Release();
	}
	closesocket(out);
	WSACleanup();
	CameraManager::X().Shutdown();
	return 0;
}