#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\videoio.hpp>
#include <opencv\highgui.h>
#include <opencv2\imgproc\imgproc.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cstdint>

#include "pxcsensemanager.h"


using namespace std;

class RealSense
{
	// Realsense camera access class, controls everything.
	PXCSenseManager *pxcsm;
	PXCCapture::Device *device;
	// Realsense types for the raw rgb and depth data and image info
	PXCImage::ImageData pxc_data_rgb;
	PXCImage::ImageData pxc_data_depth;
	PXCImage::ImageData pxc_data_ir;

	PXCImage::PixelFormat depth_type;

	bool depth_flag;
	bool ir_flag;
	bool colour_flag;
public:
	bool parallel;
	bool laser;
	cv::Mat mat_rgb;
	cv::Mat mat_depth, mat_depth1;
	cv::Mat mat_ir, mat_ir1;
	bool setup(bool depth_flag, bool ir_flag, bool colour_flag, bool depth_raw = true, bool parallel = false);
	bool doCapture();
	int queryLaser();
	int switchLaser();
	int setLaser(bool value);
	bool saveFrame(string file_name_in);
	bool thresholdIrFrame(int threshold_value, int threshold_type);
	bool isDepthRaw() { return (depth_type == PXCImage::PixelFormat::PIXEL_FORMAT_DEPTH_RAW); }
	RealSense();
	~RealSense();
};

