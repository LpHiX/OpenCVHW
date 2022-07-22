#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "cameralibrary.h"

using namespace CameraLibrary;
using namespace std;
using namespace cv;

/// Importing images

void test()
{

}

int main() {
	cout << "Main Program Starting" << endl;
	
	CameraLibrary_EnableDevelopment();
	CameraManager::X().WaitForInitialization();
	
	
	
	Camera *camera = CameraManager::X().GetCamera();
	if (!camera) {
		cout << "Camera is invalid" << endl;
		return 0;
	}

	cout << "Camera found, Starting loop" << endl;

	const int cameraWidth = 2048;
	const int cameraHeight = 1088;


	camera->SetVideoType(Core::MJPEGMode);
	camera->SetAEC(true);
	camera->SetAGC(true);
	camera->SetTextOverlay(true);

	camera->Start();

	//Surface Texture(cameraWidth, cameraHeight);
	//Bitmap* framebuffer = new Bitmap(cameraWidth, cameraHeight, Texture.PixelSpan() * 4, Bitmap::ThirtyTwoBit, Texture.GetBuffer());
	char imageBuffer[cameraWidth * cameraHeight];
	//Mat1b imgBuffer(2048, 1088);

	Mat img;
	//img = imread("Resources/test.png");
	//imshow(" hi", img);

	while(1)
	{
		Frame *frame = camera->GetFrame();
		if (frame) {

			frame->Rasterize(cameraWidth, cameraHeight, cameraWidth, 8, imageBuffer);
			img = Mat(cameraHeight, cameraWidth, CV_8UC1, &imageBuffer);
			imshow("hi", img);
			
			char key = (char)cv::waitKey(1);
			if (key == 27) break;

			//cout << imageBuffer[1000] << endl;
			frame->Release();
		}
	}
	cout << "Closing down" << endl;
	camera->Release();
	CameraManager::X().Shutdown();
	return 0;
}