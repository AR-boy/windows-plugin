
#pragma
#include "opencv2/objdetect.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "FindMarkers.h"
#include <iostream>
#include <stdio.h>

//using namespace std;
//using namespace cv;

// Declare structure to be used to pass data from C++ to Mono.
struct Circle
{
	Circle(int x, int y, int radius) : X(x), Y(y), Radius(radius) {}
	int X, Y, Radius;
};


struct UnityPoint
{
	UnityPoint(float x_param, float y_param) : x(x_param), y(y_param) {}
	float x, y;
};
/*
struct Marker
{
	int markerNumber;
	std::vector<std::vector<cv::Point2f>> list;
	MarkerList(int numOfMarkers, std::vector<std::vector<cv::Point2f>> markers)
	{
		markerNumber = numOfMarkers;
		list = markers;
	}
};

*/


struct MarkerList
{
	std::vector<std::vector<cv::Point2f>> list;
	MarkerList(int numOfMarkers, std::vector<std::vector<cv::Point2f>> markers)
	{
		list = markers;
	}
};



struct DetectedMarkers
{
	std::vector<std::vector<UnityPoint>> markers;
	DetectedMarkers(std::vector<std::vector<UnityPoint>> markersFound)
	{
		markers = markersFound;
	}
};

cv::CascadeClassifier _faceCascade;
cv::String _windowName = "Unity OpenCV Interop Sample";
cv::VideoCapture _capture;

int _scale = 1;



std::vector<cv::Point2f> flattenedVec;

extern "C" int __declspec(dllexport) __stdcall  Init(int& outCameraWidth, int& outCameraHeight)
{
	// Open the stream.
	_capture.open(0);
	if (!_capture.isOpened())
		return -2;

	outCameraWidth = _capture.get(cv::CAP_PROP_FRAME_WIDTH);
	outCameraHeight = _capture.get(cv::CAP_PROP_FRAME_HEIGHT);

	return 0;
}

extern "C" void __declspec(dllexport) __stdcall  Close()
{
	_capture.release();
}

extern "C" void __declspec(dllexport) __stdcall deleteMarkerPointer(std::vector<std::vector<cv::Point2f>>* detectMarkerPointer)
{
	delete detectMarkerPointer;
}


/*

cv::Point2f* SerialiseDetectCorners(std::vector<std::vector<cv::Point2f>> pointVector)
{
	int total_size = 0;
	for (auto& vec : pointVector) total_size += vec.size();
	std::vector<cv::Point2f> flattenedVec;
	flattenedVec.reserve(total_size);

	for (auto& vec : pointVector)
		for (auto& elem : vec)
			flattenedVec.push_back(elem);
	auto ptr = flattenedVec.data();
	return ptr;
}

*/



extern "C"  __declspec(dllexport)  cv::Point2f* __stdcall  GetDetectMarkerData(std::vector<cv::Point2f>* vecPointer)
{
	return vecPointer->data();
}

extern "C"  __declspec(dllexport) int __stdcall GetDetectMarkersLength(std::vector<cv::Point2f>* vecPointer)
{
	return vecPointer->size();
}

extern "C" __declspec(dllexport) std::vector<cv::Point2f>* __stdcall FindMarkers(unsigned char* textureData, int width, int height)
{

	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	cv::Mat texture = cv::Mat(height, width, CV_8UC4, textureData);
	if (texture.empty())
	{
		// return 0;
	}

	// format image for detection
	cv::cvtColor(texture, texture, cv::COLOR_BGRA2RGB);
	cv::flip(texture, texture, 0);

	// detect markers 
	cv::aruco::detectMarkers(texture, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


	flattenedVec.clear();
	if (markerCorners.empty())
	{
		return &flattenedVec;
	}
	// return 1;
	int total_size = 0;
	for (auto& vec : markerCorners) total_size += vec.size();

	flattenedVec.reserve(total_size);

	for (auto& vec : markerCorners)
		for (auto& elem : vec)
			flattenedVec.push_back(elem);

	std::vector<cv::Point2f>* flattenedP;
	// 4 bytes per 
	//return &flattenedVec;
	flattenedP = &flattenedVec;
	return flattenedP;
	// serialise data
	// std::vector<cv::Point2f>* markersP;


	/*
	for (size_t i = 0; i < markerCorners.size(); i++)
	{

		if (markerCorners[i].empty())
		{
			return 3;
		}
		for (size_t j = 0; j < markerCorners[i].size(); j++)
		{

			for (int k = 0; k < 2; k++)
			{
				if (k == 0)
				{
					// needs proper rounding
					markerList[a] = int(markerCorners[i][j].x + 0.5);
				}
				else
				{
					// needs proper rounding
					markerList[a] = int(markerCorners[i][j].y + 0.5);
				}
				a++;
			}
		}
	}
	*/
	/*
	for (size_t i = 0; i < markerCorners.size(); i++)
	{
		for (size_t j = 0; j < markerCorners[i].size(); j++)
		{
			markerList[i].corners[j] = UnityPoint(markerCorners[i][j].x, markerCorners[i][j].y);
		}

	}
	*/
}

extern "C" void __declspec(dllexport) __stdcall FillIntArray(int a[], int length)
{
	for (int i = 0; i < length; i++)
	{
		a[i] = i;
	}
}

extern "C" int __declspec(dllexport) __stdcall  TextureToMat(unsigned char* textureData, int width, int height)
{

	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	cv::Mat texture = cv::Mat(height, width, CV_8UC4, textureData);
	if (texture.empty())
	{
		return 0;
	}

	cv::cvtColor(texture, texture, cv::COLOR_BGRA2RGB);

	//cv::namedWindow(_windowName, cv::WINDOW_NORMAL);
	//cvResizeWindow("Unity Texture", 200, 200);
	//cv::imshow(_windowName, texture);
	//cv::waitKey(1);
	//cv::destroyAllWindows();

	return 1;
}

extern "C" void __declspec(dllexport) __stdcall SetScale(int scale)
{
	_scale = scale;
}

// extern "C" void __declspec(dllexport) __stdcall DetectMarkers(cv::Mat frame)
// {
//	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	// detect markers 
//	cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
//}

extern "C" int __declspec(dllexport) __stdcall DrawDetectMarkers()
{
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2i>> markerCorners, rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Mat frame;
	std::ostringstream vts;
	_capture >> frame;
	if (frame.empty())
		return 0;

	// detect markers 
	cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

	if (markerIds.empty())
		return 1;
		// Convert all but the last element to avoid a trailing "," 
	std::copy(markerIds.begin(), markerIds.end() - 1,
	std::ostream_iterator<int>(vts, ", "));

		// Now add the last element with no delimiter 
	vts << markerIds.back();
	std::string markerString = vts.str();

	cv::Mat outputImage = frame.clone();
	cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	imshow(_windowName, outputImage);

	return 3;
}

extern "C" void __declspec(dllexport) __stdcall Detect(Circle* outFaces, int maxOutFacesCount, int& outDetectedFacesCount)
{
	cv::Mat frame;
	_capture >> frame;
	if (frame.empty())
		return;

	std::vector<cv::Rect> faces;
	// Convert the frame to grayscale for cascade detection.
	cv::Mat grayscaleFrame;
	cvtColor(frame, grayscaleFrame, cv::COLOR_BGR2GRAY);
	cv::Mat resizedGray;
	// Scale down for better performance.
	resize(grayscaleFrame, resizedGray, cv::Size(frame.cols / _scale, frame.rows / _scale));
	cv::equalizeHist(resizedGray, resizedGray);

	// Detect faces.
	_faceCascade.detectMultiScale(resizedGray, faces);

	// Draw faces.
	for (size_t i = 0; i < faces.size(); i++)
	{
		cv::Point center(_scale * (faces[i].x + faces[i].width / 2), _scale * (faces[i].y + faces[i].height / 2));
		ellipse(frame, center, cv::Size(_scale * faces[i].width / 2, _scale * faces[i].height / 2), 0, 0, 360, cv::Scalar(0, 0, 255), 4, 8, 0);

		// Send to application.
		outFaces[i] = Circle(faces[i].x, faces[i].y, faces[i].width / 2);
		outDetectedFacesCount++;

		if (outDetectedFacesCount == maxOutFacesCount)
			break;
	}

	// Display debug output.
	cv::imshow(_windowName, frame);
}

int main()
{
	std::cout << "This is main function";
}