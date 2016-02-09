#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\videoio.hpp>
#include <opencv\highgui.h>

#include <stdlib.h>
#include <stdio.h>

#define TARGET_CHESSBOARD 1
#define TARGET_CIRCLES 2
#define TARGET_ASYMMETRIC_CIRCLES 3

using namespace std;

class Calibration
{
	// Chessboard x and y sizes (rows * columns)
	int tg_x;
	int tg_y;
	// Number of corners (x * y)
	int tg_n;
	// Either chessboard or symmetric circles or asymmetric circles
	int target_type;
	// Size in mm of square edge
	float tg_square_size;
	// vector of vector of chessboard corner locations as seen in sample images
	vector<vector<cv::Point2f>> image_points_vec;
	// Vector of vector of chessboard corner real positions
	vector<vector<cv::Point3f>> tg_points_vec;
	// The matrices spat out by from the calibration
	cv::Mat map1, map2;
	// The expected size of input images. Useful for checking & required for calib
	cv::Size image_size;

public:
	// minimum number of samples required for a calibration to be performed
	int min_req_samples;
	// Number of samples
	int samples;
	// Initialised and completed flags
	bool calibration_initialised;
	bool calibration_completed;
	// Some arbitrary name for the calibration, used in saving and loading calibration files.
	string calib_name;

	Calibration();
	~Calibration();

	void initCalibration(int x_dims, int y_dims, float square_size, int target_type,
		cv::Size image_size, string calib_name, int min_req_samples);
	bool addCalibrationImage(cv::Mat image);
	bool performCalibration();
	cv::Mat remapImage(cv::Mat input_image);
};

