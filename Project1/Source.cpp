#pragma
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <vector>
#include <cctype>
#include <string.h>
#include <time.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "FindMarkers.h"



// native methods that help managed c# code to marshall - cv::Mat structure

extern "C" __declspec(dllexport) bool * __stdcall CreateBooleanPointer()
{
	return new bool();
}

extern "C" void __declspec(dllexport) __stdcall DeleteBooleanPointer(bool* pointer)
{
	delete pointer;
}


extern "C" __declspec(dllexport) cv::Mat * __stdcall CreateMatPointer()
{
	return new cv::Mat();
}

extern "C" void __declspec(dllexport) __stdcall DeleteMatPointer(cv::Mat * pointer)
{
	delete pointer;
}

extern "C"  __declspec(dllexport) float __stdcall GetMatPointerOfDoubleDataAt(cv::Mat* pointer, int row, int column)
{
	return pointer->at<double>(row, column);
}
extern "C"  __declspec(dllexport) int __stdcall GetMatPointerOfIntDataAt(cv::Mat * pointer, int row, int column)
{
	return pointer->at<int>(row, column);
}


extern "C"  __declspec(dllexport) int __stdcall GetMatPointerRowNum(cv::Mat * pointer)
{
	return pointer->rows;
}

extern "C"  __declspec(dllexport) int __stdcall GetMatPointerColNum(cv::Mat * pointer)
{
	return pointer->cols;
}
extern "C"  __declspec(dllexport) int __stdcall GetMatPointerDimNum(cv::Mat * pointer)
{
	return pointer->dims;
}


// native methods that help managed c# code to marshall - std::vector<int> structure

extern "C" __declspec(dllexport) std::vector<int> * __stdcall CreateVectorIntPointer()
{
	 return new std::vector<int>();
}

extern "C" void __declspec(dllexport) __stdcall DeleteVectorIntPointer(std::vector<int> * pointer)
{
	delete pointer;
}

extern "C"  __declspec(dllexport)  int * __stdcall  GetVectorIntData(std::vector<int> * pointer)
{
	return pointer->data();
}

extern "C"  __declspec(dllexport) int __stdcall GetVectorIntSize(std::vector<int> * pointer)
{
	return pointer->size();
}



// native methods that help managed c# code to marshall -  std::vector<cv::Vec3d> structure

extern "C" __declspec(dllexport) std::vector<cv::Vec3d> * __stdcall CreateVectorVec3dPointer()
{
	return new std::vector<cv::Vec3d>();
}

extern "C" void __declspec(dllexport) __stdcall DeleteVectorVec3dPointer(std::vector<cv::Vec3d> * pointer)
{
	delete pointer;
}

extern "C"  __declspec(dllexport)  cv::Vec3d * __stdcall  GetVectorVec3dPointerData(std::vector<cv::Vec3d> * pointer)
{
	return pointer->data();
}

extern "C"  __declspec(dllexport) int __stdcall GetVectorVec3dPointerSize(std::vector<cv::Vec3d> * pointer)
{
	return pointer->size();
}


// native methods that help managed c# code to marshall - std::vector<cv::Point2f> structure

extern "C" __declspec(dllexport) std::vector<cv::Point2f> * __stdcall CreateVector2PointFPointer()
{
	return new std::vector<cv::Point2f>();
}

extern "C" void __declspec(dllexport) __stdcall DeleteVector2PointFPointer(std::vector<cv::Point2f>* pointer)
{
	delete pointer;
}

extern "C"  __declspec(dllexport)  cv::Point2f * __stdcall  GetVector2PointFPointerData(std::vector<cv::Point2f> * pointer)
{
	return pointer->data();
}

extern "C"  __declspec(dllexport) int __stdcall GetVector2PointFPointerSize(std::vector<cv::Point2f> * pointer)
{
	return pointer->size();
}


// native methods that help managed c# code to marshall - std::vector<std::vector<cv::Point2f>> structure

extern "C" __declspec(dllexport) std::vector<std::vector<cv::Point2f>> * __stdcall CreateDoubleVector2PointFPointer()
{
	return new std::vector<std::vector<cv::Point2f>>();
}

extern "C" void __declspec(dllexport) __stdcall DeleteDoubleVector2PointFPointer(std::vector<std::vector<cv::Point2f>> * pointer)
{
	delete pointer;
}

extern "C"  __declspec(dllexport)  std::vector<cv::Point2f> * __stdcall  GetDoubleVector2PointFPointerData(std::vector<std::vector<cv::Point2f>> * pointer)
{
	return pointer->data();
}
extern "C"  __declspec(dllexport)  std::vector<cv::Point2f> * __stdcall  GetDoubleVector2PointFPointerDataAt(std::vector<std::vector<cv::Point2f>> * pointer, int offset)
{
	return pointer->data()+ offset;
}

extern "C"  __declspec(dllexport) int __stdcall GetDoubleVector2PointFPointerSize(std::vector<std::vector<cv::Point2f>> * pointer)
{
	return pointer->size();
}


// native methods that help managed c# code to marshall - std::vector<cv::Point3f> structure

extern "C" __declspec(dllexport) std::vector<cv::Point3f> * __stdcall CreateVector3PointFPointer()
{
	return new std::vector<cv::Point3f>();
}
extern "C" void __declspec(dllexport) __stdcall DeleteVector3PointFPointer(std::vector<cv::Point3f> * pointer)
{
	delete pointer;
}

extern "C"  __declspec(dllexport)  cv::Point3f * __stdcall  GetVector3PointFPointerData(std::vector<cv::Point3f> * pointer)
{
	return pointer->data();
}

extern "C"  __declspec(dllexport) int __stdcall GetVector3PointFPointerSize(std::vector<cv::Point3f> * pointer)
{
	return pointer->size();
}


extern "C" __declspec(dllexport) std::vector<std::vector<cv::Point3f>> * __stdcall CreateDoubleVector3PointFPointer()
{
	return new std::vector<std::vector<cv::Point3f>>();
}

extern "C" void __declspec(dllexport) __stdcall DeleteDoubleVector3PointFPointer(std::vector<std::vector<cv::Point3f>> * pointer)
{
	delete pointer;
}

extern "C"  __declspec(dllexport)  std::vector<cv::Point3f> * __stdcall  GetDoubleVector3PointFPointerData(std::vector<std::vector<cv::Point3f>> * pointer)
{
	return pointer->data();
}
extern "C"  __declspec(dllexport)  std::vector<cv::Point3f> * __stdcall  GetDoubleVector3PointFPointerDataAt(std::vector<std::vector<cv::Point3f>> * pointer, int offset)
{
	return pointer->data() + offset;
}

extern "C"  __declspec(dllexport) int __stdcall GetDoubleVector3PointFPointerSize(std::vector<std::vector<cv::Point3f>> * pointer)
{
	return pointer->size();
}



extern "C" __declspec(dllexport) void __stdcall DetectMarkers(
	unsigned char* textureData, 
	int width, 
	int height, 
	std::vector<int> * markerIds,
	std::vector<std::vector<cv::Point2f>> * markerCorners,
	std::vector<std::vector<cv::Point2f>> * rejectedCandidates)
{
	// assign pointers
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	cv::Mat texture = cv::Mat(height, width, CV_8UC4, textureData);
	if (texture.empty())
	{
		// return markerCorners;
	}

	// format image for detection
	cv::cvtColor(texture, texture, cv::COLOR_BGRA2RGB);
	cv::flip(texture, texture, 0);

	// detect markers
	cv::aruco::detectMarkers(texture, dictionary, *markerCorners, *markerIds, parameters, *rejectedCandidates);

	if (markerCorners->empty())
	{
		// return markerCorners;
	}
	// return pointer
	// return markerCorners;
	// later return an int for exit code
}	

void NDetectMarkers(
	cv::Mat texture,
	std::vector<int>* markerIds,
	std::vector<std::vector<cv::Point2f>>* markerCorners,
	std::vector<std::vector<cv::Point2f>>* rejectedCandidates
)
{
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	if (texture.empty())
	{
	}

	// format image for detection
	// detect markers
	cv::aruco::detectMarkers(texture, dictionary, *markerCorners, *markerIds, parameters, *rejectedCandidates);

	if (markerCorners->empty())
	{
	}
	// return pointer

}

/*


void NCalibrateCamera(
	int width,
	int height,
	float squareLength,
	float markerLength,
	std::vector<std::vector<int>>* allCharucoIds,
	std::vector<std::vector<cv::Point2f>>* allCharucoCorners
)
{
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> tvecs;
	std::vector<cv::Mat> rvecs;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(7, 9, squareLength, markerLength, dictionary);
	cv::Size imgSize = cv::Size(width, height);

	// Detect charuco board from several viewpoints and fill allCharucoCorners and allCharucoIds

	// After capturing in several viewpoints, start calibration
	int calibrationFlags = 0;
	// Set calibration flags (same than in calibrateCamera() function)
	double repError = cv::aruco::calibrateCameraCharuco(*allCharucoCorners, *allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

	std::cout << "finished calibrate\n";
}

*/




/*
int main()
{
	// GLOBAL
	cv::String _windowName = "Unity OpenCV Interop Sample";
	cv::VideoCapture _capture;
	cv::Mat frame;
	std::vector<std::vector<int>>* allCharucoIds = new std::vector<std::vector<int>>();
	std::vector<std::vector<cv::Point2f>>* allCharucoCorners = new std::vector<std::vector<cv::Point2f>>();
	cv::Mat* cameraMatrix = new cv::Mat();
	cv::Mat* distCoeffs = new cv::Mat();
	std::vector<cv::Mat>* tvecs = new std::vector<cv::Mat>();
	std::vector<cv::Mat>* rvecs = new std::vector<cv::Mat>();

	bool calibrating = true;
	bool notCalibrated = true;

	_capture.open(0);
	if (!_capture.isOpened())
		return -2;

	while (true)
	{
		_capture >> frame;
		if (frame.empty())
		{

		} 
		else
		{
			auto start = std::chrono::steady_clock::now();
			auto end = std::chrono::steady_clock::now();
			while (calibrating == true)
			{
				std::cout << "runnign calibrate\n";
				std::vector<int>* markerIds = CreateVectorIntPointer();
				std::vector<std::vector<cv::Point2f>>* markerCorners = CreateDoubleVector2PointFPointer();
				std::vector<std::vector<cv::Point2f>>* rejectedCandidates = CreateDoubleVector2PointFPointer();

				NDetectMarkers(frame, markerIds, markerCorners, rejectedCandidates);
				
				std::vector<int> frameMarkerIds = *markerIds;
				std::vector<std::vector<cv::Point2f>> frameMarkerCorners = *markerCorners;

				allCharucoIds->push_back(frameMarkerIds);
				allCharucoCorners->insert(allCharucoCorners->end(), frameMarkerCorners.begin(), frameMarkerCorners.end());

				DeleteDoubleVector2PointFPointer(markerCorners);
				DeleteDoubleVector2PointFPointer(rejectedCandidates);
				DeleteVectorIntPointer(markerIds);

				cv::imshow(_windowName, frame);
				end = std::chrono::steady_clock::now();
				cv::waitKey(1);
				std::chrono::milliseconds elsapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
				if (elsapsedTime.count() > 5000)
				{
					calibrating = false;
				}
				_capture >> frame;
			}
			if (notCalibrated == true)
			{

				NCalibrateCamera(
					frame.cols,
					frame.rows,
					0.025,
					0.0125,
					allCharucoIds,
					allCharucoCorners
				);

				notCalibrated = false;
			}
			cv::Mat undistorted;

			cv::undistort(frame, undistorted, *cameraMatrix, *distCoeffs);

			cv::imshow(_windowName, frame);
		}
		cv::waitKey(1);
	}

	return 0;
}
*/
/*
int main(int argc) {

	// globals
    int squaresX = 7;
    int squaresY = 5;
    float squareLength = 0.025;
    float markerLength = 0.0125;
    int dictionaryId = cv::aruco::DICT_6X6_250;
    int calibrationFlags = 0;


    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();


    cv::VideoCapture inputVideo;
	inputVideo.open(0);
	if (!inputVideo.isOpened())
		return -2;

    cv::Ptr<cv::aruco::Dictionary> dictionary =cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // create charuco board object
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();

    // collect data from each frame
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
	std::vector<std::vector<int>> allIds;
	std::vector<cv::Mat> allImgs;
    cv::Size imgSize;

	bool thisAlive = true;
    while (thisAlive) {
        cv::Mat image, imageCopy;
		inputVideo >> image;

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners, rejected;

        // detect markers
       cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        //if (refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // interpolate charuco corners
        cv::Mat currentCharucoCorners, currentCharucoIds;
        if (ids.size() > 0)
            cv::aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
                currentCharucoIds);

        // draw results
        image.copyTo(imageCopy);
		if (ids.size() > 0)
		{
			cv::aruco::drawDetectedMarkers(imageCopy, corners);
		}

		if (currentCharucoCorners.total() > 0)
		{
			cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
		}
        putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
            cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

        imshow("out", imageCopy);
		char key = (char)cv::waitKey(10);
		if (key == 27) thisAlive = false;
        if (key == 'c' && ids.size() > 0) {
            std::cout << "Frame captured" << std::endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(image);
            imgSize = image.size();
        }
    }

    if (allIds.size() < 1) {
        std::cerr << "Not enough captures for calibration" << std::endl;
        return 0;
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double repError;

	// fish eye cam set up is bad
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // prepare data for calibration
    std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
	std::vector<int> allIdsConcatenated;
	std::vector<int> markerCounterPerFrame;

    markerCounterPerFrame.reserve(allCorners.size());
    for (unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for (unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, board, imgSize, cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), calibrationFlags);

    // prepare data for charuco calibration
    int nFrames = (int)allCorners.size();
    std::vector<cv::Mat> allCharucoCorners;
	std::vector<cv::Mat> allCharucoIds;
	std::vector<cv::Mat> filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);

    for (int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        cv::Mat currentCharucoCorners, currentCharucoIds;
        cv::aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard, currentCharucoCorners, currentCharucoIds, cameraMatrix, distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }

    if (allCharucoCorners.size() < 4) {
		std::cerr << "Not enough corners for calibration" << std::endl;
        return 0;
    }

    // calibrate camera using charuco
    repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

    std::cout << "Rep Error: " << repError << std::endl;
	std::cout << "Rep Error Aruco: " << arucoRepErr << std::endl;

	bool keepAlive = true;
    // show interpolated charuco corners for debugging
	while (keepAlive)
	{

		cv::Mat image, rview, map1, map2;
		/*
				inputVideo.open(0);
		if (!inputVideo.isOpened())
			std::cout << "close camera: " << repError << std::endl;
			return -2;
		*/

        /*
		inputVideo >> image;
		if (!image.empty())
		{
			cv::Mat newCamMat;
			int dTOatal = distCoeffs.total();
			bool dEpmty = distCoeffs.empty();
			initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
				getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imgSize, 1, imgSize, 0), imgSize,
				CV_16SC2, map1, map2);
			cv::remap(image, rview, map1, map2, cv::INTER_LINEAR);
			imshow("Image View", rview);
		}

		char key = (char)cv::waitKey(10);
		if (key == 27) break;

	}

		for (unsigned int frame = 0; frame < filteredImages.size(); frame++) {
        cv::Mat imageCopy = filteredImages[frame].clone();
        if (allIds[frame].size() > 0) {

            if (allCharucoCorners[frame].total() > 0) {
				cv::aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame],
                    allCharucoIds[frame]);
            }
        }

		cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(0);
        if (key == 27) break;
    }
	
    
    return 0;
}
*/


 void NCalibrateCamera(
	cv::Mat texture,
	int width,
	int height,
	std::vector<std::vector<cv::Point2f>> * image_points,
	std::vector<std::vector<cv::Point3f>> * object_points,
	cv::Mat * cameraMatrix,
	cv::Mat * distCoeffs
)
{

	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;

	*cameraMatrix = cv::Mat(3, 3, CV_32FC1);
	cameraMatrix->ptr<float>(0)[0] = 1;
	cameraMatrix->ptr<float>(1)[1] = 1;


	cv::calibrateCamera(*object_points, *image_points, texture.size(), *cameraMatrix, *distCoeffs, rvecs, tvecs);
}

void NFindChessboardCorners(
	cv::Mat texture,
	int width,
	int height,
	int cornersW,
	int cornersH,
	float cornerLength,
	float cornerSeparation,
	bool* foundBoardMarkers,
	std::vector<std::vector<cv::Point2f>> * image_points,
	std::vector<std::vector<cv::Point3f>> * object_points
)
{
	int numSquares = cornersH * cornersW;
	cv::Mat gray_texture;
	cv::Size board_sz = cv::Size(cornersH, cornersW);
	std::vector<cv::Point3f> obj;
	std::vector<cv::Point2f> corners;


	// format image for detection
	cv::cvtColor(texture, gray_texture, cv::COLOR_BGR2GRAY);


	for (int j = 0; j < numSquares; j++)
	{
		obj.push_back(cv::Point3f(j / cornersH, j % cornersH, 0.0f));
	}

	bool found = findChessboardCorners(gray_texture, board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
	if (found)
	{
		cornerSubPix(gray_texture, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
		image_points->push_back(corners);
		object_points->push_back(obj);
		*foundBoardMarkers = true;
	}

}


extern "C" __declspec(dllexport) void __stdcall  EstimateSingleMarkerPose(
	float markerLength,
	std::vector<std::vector<cv::Point2f>> * markerCorners,
	cv::Mat * cameraMatrix,
	cv::Mat * distCoeffs,
	std::vector<cv::Vec3d> * rvecs,
	std::vector<cv::Vec3d> * tvecs
)
{
	cv::aruco::estimatePoseSingleMarkers(*markerCorners, markerLength, *cameraMatrix, *distCoeffs, *rvecs, *tvecs);
}
/*

extern "C" __declspec(dllexport) void __stdcall  EstimateBoardPose(
	std::vector<std::vector<int>> * detectedCornerIds,
	std::vector<std::vector<cv::Point2f>> * detectedCorners,
	int width,
	int height,
	int cornersW,
	int cornersH,
	float cornerLength,
	float cornerSeparation,
	cv::Mat * cameraMatrix,
	cv::Mat * distCoeffs,
	cv::Vec3d * rvec,
	cv::Vec3d * tvec
)

{
	cv::aruco::estimatePoseBoard(
		*detectedCorners, 
		*detectedCornerIds, 
		_board,
		*cameraMatrix, 
		*distCoeffs, 
		*rvec, 
		*tvec
	);
}
*/
extern "C" __declspec(dllexport) void __stdcall  FindChessboardCorners(
	unsigned char* textureData,
	int width,
	int height,
	int cornersW,
	int cornersH,
	float cornerLength, 
	float cornerSeparation,
	bool * foundBoardMarkers,
	std::vector<std::vector<cv::Point2f>> * image_points, 
	std::vector<std::vector<cv::Point3f>> * object_points
)
{
	int numSquares = cornersH * cornersW;
	cv::Mat texture = cv::Mat(height, width, CV_8UC4, textureData);
	cv::Mat gray_texture;
	cv::Size board_sz = cv::Size(cornersH, cornersW);
	std::vector<cv::Point3f> obj;
	std::vector<cv::Point2f> corners;

	
	// format image for detection
	cv::flip(texture, texture, 0);
	cv::cvtColor(texture, texture, cv::COLOR_BGRA2RGB);
	cvtColor(texture, gray_texture, cv::COLOR_RGB2GRAY);

	for (int j = 0; j < numSquares; j++)
	{
		obj.push_back(cv::Point3f(j / cornersH, j % cornersH, 0.0f));
	}


	bool found = findChessboardCorners(texture, board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

	if(found) 
	{
		cornerSubPix(gray_texture, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
		image_points->push_back(corners);
		object_points->push_back(obj);
		*foundBoardMarkers = true;
	}



}

extern "C" __declspec(dllexport) void __stdcall  CalibrateCamera(
	unsigned char* textureData,
	int height,
	int width,
	std::vector<std::vector<cv::Point2f>> * image_points,
	std::vector<std::vector<cv::Point3f>> * object_points,
	cv::Mat * cameraMatrix,
	cv::Mat * distCoeffs
)
{

	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	
	cv::Mat texture = cv::Mat(height, width, CV_8UC4, textureData);
	cv::cvtColor(texture, texture, cv::COLOR_BGRA2RGB);

	*cameraMatrix = cv::Mat(3, 3, CV_32FC1);
	cameraMatrix->ptr<float>(0)[0] = 1;
	cameraMatrix->ptr<float>(1)[1] = 1;


	cv::calibrateCamera(*object_points, *image_points, texture.size(), *cameraMatrix, *distCoeffs, rvecs, tvecs);
}


extern "C" __declspec(dllexport) void __stdcall SaveCameraParams(
	std::string* filename,
	int imageWidth,
	int imageHeight,
	int cornersH,
	int cornersW,
	float cornerLength,
	float cornerSeparation,
	const cv::Mat& cameraMatrix, 
	const cv::Mat& distCoeffs
)
{
	cv::FileStorage fs(*filename, cv::FileStorage::WRITE);

	fs << "image_width" << imageWidth;
	fs << "image_height" << imageHeight;
	fs << "board_width" << cornersW;
	fs << "board_height" << cornersH;
	fs << "corner_size" << cornerLength;
	fs << "corner_seperation" << cornerSeparation;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
}
extern "C" __declspec(dllexport) void __stdcall ReadSavedParams(
	const std::string & filename,
	int* imageWidth,
	int* imageHeight,
	int* cornersW,
	int* cornersH,
	float* cornerLength,
	float* cornerSeparation,
	cv::Mat * cameraMatrix,
	cv::Mat* distCoeffs
) {
	cv::FileStorage fs;
	fs.open(filename, cv::FileStorage::READ);

	fs["image_width"] >> *imageWidth;
	fs["image_height"] >> *imageHeight;
	fs["board_width"] >> *cornersW;
	fs["board_height"] >> *cornersH;
	fs["corner_size"] >> *cornerLength;
	fs["corner_seperation"] >> *cornerSeparation;

	fs["camera_matrix"] >> *cameraMatrix;
	fs["distortion_coefficients"] >> *distCoeffs;
}

std::string type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

void testCalibtration()
{

	int cornersH = 9;
	int cornersW = 6;
	float cornerLength = 0.024;
	float cornerSeparation = 0.024;
	float sizeOfCorner = 0.045;
	int sucessfulFrameNum = 4;

	cv::VideoCapture capture = cv::VideoCapture(0);

	std::vector<std::vector<cv::Point3f>>* object_points = CreateDoubleVector3PointFPointer();
	std::vector<std::vector<cv::Point2f>>* image_points = CreateDoubleVector2PointFPointer();
	cv::Mat* camera_matrix = CreateMatPointer();
	cv::Mat* dist_coeffs = CreateMatPointer();


	cv::Mat image;
	capture >> image;

	int imageWidth = image.cols;
	int imageHeigth = image.rows;

	int successes = 0;
	while (successes < sucessfulFrameNum)
	{
		capture >> image;

		bool * foundBoardMarkers= new bool;
		*foundBoardMarkers = false;

		cv::imshow("win1", image);
		int key = cv::waitKey(1);

		if (key == ' ')
		{

			printf("trying to capture image!");
			NFindChessboardCorners(
				image,
				imageWidth,
				imageHeigth,
				cornersW,
				cornersH,
				cornerLength,
				cornerSeparation,
				foundBoardMarkers,
				image_points,
				object_points
			);

			if (*foundBoardMarkers)
			{

				printf("Snap stored!");
				successes++;
			}
			else
			{
				printf("failed!");
			}

		}

		delete foundBoardMarkers;
	}

	NCalibrateCamera(
		image,
		imageWidth,
		imageHeigth,
		image_points,
		object_points,
		camera_matrix,
		dist_coeffs
	);

	double test = GetMatPointerOfDoubleDataAt(dist_coeffs, 0, 2);
	std::cout << type2str(dist_coeffs->type()) << std::endl;
	std::cout << type2str(camera_matrix->type()) << std::endl;
 	std::cout << test << std::endl;
	std::cout << *dist_coeffs << std::endl;
	std::cout << *camera_matrix << std::endl;
	bool keepAlive = true;
	// show interpolated charuco corners for debugging
	while (keepAlive)
	{
		capture >> image;

		if (!image.empty())
		{
			std::vector<cv::Vec3d> rvecs, tvecs;
			std::vector<int>* markerIds = CreateVectorIntPointer();
			std::vector<std::vector<cv::Point2f>>* markerCorners = CreateDoubleVector2PointFPointer();
			std::vector<std::vector<cv::Point2f>>* rejectedCandidates = CreateDoubleVector2PointFPointer();

			NDetectMarkers(
				image,
				markerIds,
				markerCorners,
				rejectedCandidates
			);


			cv::aruco::estimatePoseSingleMarkers(*markerCorners, sizeOfCorner, *camera_matrix, *dist_coeffs, rvecs, tvecs);

			for (int i = 0; i < rvecs.size(); ++i) {
				auto rvec = rvecs[i];
				auto tvec = tvecs[i];
				cv::aruco::drawAxis(image, *camera_matrix, *dist_coeffs, rvec, tvec, 0.1);
			}
			cv::imshow("win1", image);
			cv::waitKey(1);

			DeleteDoubleVector2PointFPointer(markerCorners);
			DeleteDoubleVector2PointFPointer(rejectedCandidates);
			DeleteVectorIntPointer(markerIds);

		}
	}

}



int main(int argc) 
{
	testCalibtration();
	return 0;
}
/*

int main(int argc) {

	int cornersH = 9;
	int cornersV = 6;
	float cornerLength = 0.024;
	float cornerSeparation = 0.024;
	int numBoards = 10;


	cv::VideoCapture capture = cv::VideoCapture(0);

	int numSquares = cornersH * cornersV;
	cv::Size board_sz = cv::Size(cornersH, cornersV);

	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;

	std::vector<cv::Point2f> corners;
	int successes = 0;

	cv::Mat image;
	cv::Mat gray_image;

	capture >> image;

	std::vector<cv::Point3f> obj;
	for (int j = 0; j < numSquares; j++)
	{
		obj.push_back(cv::Point3f(j / cornersH, j % cornersH, 0.0f));
	}


	while(successes < numBoards)
	{
		cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

		bool found = findChessboardCorners(image, board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

		if (found)
		{
			cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		cv::imshow("win1", image);
		cv::imshow("win2", gray_image);

		capture >> image;
		int key = cv::waitKey(1);

		if (key == 27)
		{
			return 0;
		}

		if (key == ' ' && found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);

			printf("Snap stored!");

			successes++;

			if (successes >= numBoards)
				break;
		}
	}

	cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	cv::calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);


	cv::Mat imageUndistorted;
	capture >> image;
	cv::Mat rview, map1, map2;
	// initUndistortRectifyMap(intrinsic, distCoeffs, cv::Mat(), getOptimalNewCameraMatrix(intrinsic, distCoeffs, image.size(), 1, image.size(), 0), image.size(), CV_16SC2, map1, map2);
	while (1)
	{
		cv::undistort(image, imageUndistorted, intrinsic, distCoeffs);
		//cv::remap(image, rview, map1, map2, cv::INTER_LINEAR);
		cv::imshow("win1", image);
		cv::imshow("win2", imageUndistorted);
		cv::waitKey(1);
		capture >> image;
	}

	capture.release();

	return 0;
}



*/
