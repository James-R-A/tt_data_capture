/*
	Code based on the following source:
	https://github.com/dthepok/realsense
	Modified by James Andrew, November 2015
*/

#include "RealSense.h"

#define SET_WIDTH 640
#define SET_HEIGHT 480

RealSense::RealSense()
{
	pxcsm = NULL;
}


bool RealSense::setup(bool depth_flag, bool ir_flag, bool colour_flag, bool depth_raw, bool parallel)
{
	this->mat_depth = cv::Mat::zeros(cv::Size(480, 640), CV_16UC1);
	this->mat_ir = cv::Mat::zeros(cv::Size(480, 640), CV_8UC1);
	this->parallel = parallel;
	bool ret_bool = false;
	this->depth_flag = depth_flag;
	this->ir_flag = ir_flag;
	this->colour_flag = colour_flag;
	this->depth_type = depth_raw ? PXCImage::PIXEL_FORMAT_DEPTH_RAW : PXCImage::PIXEL_FORMAT_DEPTH;
	// Sets up an instance of PXCSenseManager, which essentially
	// controls the realsense camera. If we can't create an instance,
	// it probably means the camera can't be found.
	pxcsm = PXCSenseManager::CreateInstance();
	
	if (!pxcsm)
	{
		ret_bool = false;
		cout << "Unable to create PXCSenseManager" << endl;
	}
	else
	{
		// Select the colour and depth streams and initialise the
		// stream samples.
		if (this->depth_flag)
			pxcsm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, SET_WIDTH, SET_HEIGHT);
		if(this->ir_flag)
			pxcsm->EnableStream(PXCCapture::STREAM_TYPE_IR, SET_WIDTH, SET_HEIGHT);
		if (this->colour_flag)
			pxcsm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, SET_WIDTH, SET_HEIGHT);
		
		pxcsm->Init();
		
		// Set up the 'device' so we can control laser power
		device = pxcsm->QueryCaptureManager()->QueryDevice();
		if (!device)
		{
			ret_bool = false;
			cout << "Unable to create PXCCapture Device" << endl;
		}
		else
		{
			ret_bool = true;
		}
		
	}

	return ret_bool;
}

bool RealSense::doCapture()
{
	bool ret_bool=false;

	if (!(pxcsm->AcquireFrame(true) < PXC_STATUS_NO_ERROR))
	{
		PXCCapture::Sample *sample = pxcsm->QuerySample();

		if (this->depth_flag)
		{
			PXCImage *depthIm;
			// fetch the frame from the sample
			depthIm = sample->depth;
			PXCImage *pxc_depth_image = depthIm;
			// Get access to the raw data
			pxc_depth_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, this->depth_type, &pxc_data_depth);
			// Shoves the pxc image data into an open cv Mat 
			// CV_8UC3 is a 3 channel 8 bit unsigned character (ie 24 bit rgb equivalent)
			// CV_16U is a 16 bit unsigned int
			cv::Mat temp(SET_HEIGHT, SET_WIDTH, CV_16UC1, (uint16_t*)pxc_data_depth.planes[0]);
			temp.copyTo(mat_depth);
			pxc_depth_image->ReleaseAccess(&pxc_data_depth);
		}

		if (this->ir_flag)
		{
			PXCImage *irIm;
			irIm = sample->ir;
			PXCImage *pxc_ir_image = irIm;
			pxc_ir_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::PIXEL_FORMAT_Y8, &pxc_data_ir);
			cv::Mat temp(SET_HEIGHT, SET_WIDTH, CV_8UC1, (uint8_t*)pxc_data_ir.planes[0]);
			temp.copyTo(mat_ir);
			pxc_ir_image->ReleaseAccess(&pxc_data_ir);
		}

		if (this->colour_flag)
		{
			PXCImage *colourIm;
			colourIm = sample->color;
			PXCImage *pxc_colour_image = colourIm;
			pxc_colour_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::PIXEL_FORMAT_RGB24, &pxc_data_rgb);
			cv::Mat temp(SET_HEIGHT, SET_WIDTH, CV_8UC3, pxc_data_rgb.planes[0]);
			temp.copyTo(mat_rgb);
			pxc_colour_image->ReleaseAccess(&pxc_data_rgb);
		}

		ret_bool = true;
		pxcsm->ReleaseFrame();
	}
	else
		ret_bool = false;

	return ret_bool;
}

int RealSense::queryLaser()
{
	return device->QueryIVCAMLaserPower();
}

int RealSense::switchLaser()
{
	int ret_power;

	// Turn laser off if it's on, and vice versa
	int set_power = (queryLaser() == 0) ? 16 : 0;
	pxcStatus stat = device->SetIVCAMLaserPower(set_power);

	// Check power set function returned successfuly.
	// If so, return new laser power, else return -1.
	if (stat == PXC_STATUS_NO_ERROR)
		ret_power = set_power;
	else
		ret_power = -1;

	return ret_power;
}

int RealSense::setLaser(bool value)
{
	int ret_power;
	int set_power = value? 16 : 0;
	pxcStatus stat = device->SetIVCAMLaserPower(set_power);

	// Check power set function returned successfuly.
	// If so, return new laser power, else return -1.
	if (stat == PXC_STATUS_NO_ERROR)
		ret_power = set_power;
	else
		ret_power = -1;

	return value;
}

bool RealSense::saveFrame(string file_name_in)
{
	bool ret_bool = false;
	string file_name = file_name_in;

	if(this->depth_flag)
		ret_bool = cv::imwrite(file_name+"depth.png", this->mat_depth);
	if (this->colour_flag)
		ret_bool = ret_bool && cv::imwrite(file_name + "colour.png", this->mat_rgb);
	if (this->ir_flag)
		ret_bool = ret_bool && cv::imwrite(file_name + "ir.png", this->mat_ir);
	
	return ret_bool;
}

bool RealSense::thresholdIrFrame(int threshold_value, int threshold_type)
{
	bool ret_bool = false;
	if (!(this->ir_flag))
		return ret_bool;
	else if (!(this->mat_ir.data))
		return ret_bool;
	
	cv::Mat dst;
	threshold(mat_ir, dst, threshold_value, 255, threshold_type);
	mat_ir = dst;
	ret_bool = true;
	return ret_bool;
}

RealSense::~RealSense()
{
	if(pxcsm)
		pxcsm->Release();
}
