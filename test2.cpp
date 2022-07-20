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


	cout << "Main Program Starting" << endl;

	CameraManager::X().WaitForInitialization();
	Camera* camera = CameraManager::X().GetCamera();
	if (!camera) {
		cout << "Camera is invalid" << endl;
		return 0;
	}

	cout << "Camera found, Starting loop" << endl;

	camera->Start();

	for (int i = 0; i < 10; i++) {
		Frame* frame = camera->GetFrame();
		if (frame) {
			cout << "Worked " + i << endl;
		}
	}
	camera->Release();
	CameraManager::X().Shutdown();
	return 0;
}