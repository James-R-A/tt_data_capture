#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\videoio.hpp>
#include <opencv\highgui.h>

#include <stdlib.h>
#include <stdio.h>
#include "Camera.h"
#include "RealSense.h"

#define TARGET_CHESSBOARD 1
#define TARGET_CIRCLES 2
#define TARGET_ASYMMETRIC_CIRCLES 3

class ParallelCapture
{
	// Size of both inputs. Should be the same.
	cv::Size image_size;

	// Target x and y internal
	int tg_x;
	int tg_y;
	// Number of internal corners (x * y)
	int tg_n;
	// Size in mm of gap between points
	float tg_square_size;
	// The type of calibration target used.
	int target_type;

	// vector of vector of chessboard corner locations as seen in sample images
	vector<vector<cv::Point2f>> image_points_vec[2];
	// Vector of vector of chessboard corner real positions
	vector<vector<cv::Point3f>> tg_points_vec;
	// The matrices spat out by from the calibration
	cv::Mat map0x, map0y, map1x, map1y;

public:
	// minimum number of samples required for a calibration to be performed
	int min_req_samples;
	// Number of samples
	int samples;
	// Initialised and completed flags
	bool calibration_initialised;
	bool calibration_completed;

	cv::Mat depth_remapped;
	cv::Mat ir_remapped;
	cv::Mat cam_remapped;

	// Initialise a calibration.
	void initCalibration(int x_dims, int y_dims, float square_size, int target_type,
		cv::Size image_size, int min_req_samples);
	// Attempt to add a sample to the current calibration, 
	// Returns true and adds sample if chessboard found in both images
	bool addCalibrationImage(cv::Mat cam_frame, cv::Mat depthcam_frame);
	// Completes the calibration process. 
	// Returns fail if not enough samples exist, or calibration not in process
	// or calibration already complete.
	bool performCalibration();
	// remaps the raw data frames using calibration results
	// Inputs can be empty if they aren't to be remapped
	// Ir and depth frames are remapped using the same transforms.
	bool remapImages(cv::Mat cam_frame, cv::Mat depthcam_ir, cv::Mat depthcam_depth);

	bool saveCalibration(string filename);
	bool loadCalibration(string path);
	ParallelCapture();
	~ParallelCapture();
};

