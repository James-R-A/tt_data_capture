#include <opencv2\opencv.hpp>
#include <opencv2\videoio.hpp>
#include <opencv\highgui.h>

#include <stdlib.h>
#include <stdio.h>

#include "yocto_api.h"
#include "yocto_relay.h"

#pragma once
class Camera
{
private:
	cv::VideoCapture capture;
	cv::Size image_size;
	YRelay *relay;
	Y_STATE_enum relay_state;

public:
	bool open;
	cv::Mat frame_raw, held_frame, prev_held_frame;
	cv::Mat frame_gray;
	std::string camera_name;
	int camera_number;
	bool relay_init;
	bool leds;

	Camera();
	~Camera();

	static std::vector<int> detectCameras(int max_check, bool builtin_flag);
	bool setup(int camera_number, std::string camera_name, cv::Size image_size);
	bool grab();
	bool doCapture(bool hold_frame = false);
	cv::Mat getFrameGrayscale(bool use_prev_held_frame = false);
	bool initRelay();
	bool switchRelay();
	bool setRelay(bool value);
};

