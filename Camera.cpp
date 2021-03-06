#include "Camera.h"
#include <Windows.h>

Camera::Camera()
{
	open = false;
	relay_init = false;
	this->relay = NULL;
}
Camera::~Camera()
{
}

std::vector<int> Camera::detectCameras(int max_check, bool builtin_flag)
{
	int i;
	int j = 0;
	cv::VideoCapture capture_ex;
	std::vector<int> return_vec;

	// If the deice has a builtin camera which we want to ignore
	if (builtin_flag)
	{
		max_check++;
		j = 1;
	}
	
	// Attempt to open a videoCapture object for each number in 
	// the range 0-max_check. If successful, return
	for (i = j; i < max_check; i++)
	{
		capture_ex = cv::VideoCapture(i);
		if (capture_ex.isOpened())
			return_vec.push_back(i);
		
		capture_ex.release();
		// Necessary so streams coming from the same physical camera are detected
		// Otherwsie they can be lost as the hardware resets.
		Sleep(1000);
	}

	return return_vec;
}

// Attempt to create a VideoCapture object for the given camera_number.
// If it opens successfully, set the objects frame dimensions from the 
// given image_size param.
bool Camera::setup(int camera_number, std::string camera_name, cv::Size image_size)
{
	bool return_bool = false;
	this->image_size = image_size;
	this->camera_name = camera_name;
	this->camera_number = camera_number; 
	this->held_frame = cv::Mat::zeros(image_size, CV_8UC3);
	this->prev_held_frame = cv::Mat::zeros(image_size, CV_8UC3);

	capture = cv::VideoCapture(camera_number);
	if (capture.isOpened())
	{
		capture.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
		capture.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width);
		std::cout << std::to_string(capture.get(CV_CAP_PROP_BUFFERSIZE)) << std::endl;
		capture.set(CV_CAP_PROP_BUFFERSIZE, 1);
		std::cout << std::to_string(capture.get(CV_CAP_PROP_BUFFERSIZE)) << std::endl;
		this->open = true;
		return_bool = true;
	}

	return return_bool;
}

// Capture a frame, store to this->frame_raw.
bool Camera::doCapture(bool hold_frame)
{
	
	bool return_bool = false;

	if (hold_frame)
		held_frame.copyTo(prev_held_frame);

	capture >> frame_raw;
	if (hold_frame)
		frame_raw.copyTo(held_frame);
	if (capture.isOpened())
		return_bool = true;

	/*if (relay_init)
	{
		setRelay(false);
	}*/

	return return_bool;
}

bool Camera::grab()
{
	capture.grab();
	capture.retrieve(frame_raw);
	return true;
}

// Convert this->frame_raw into a grayscale image and return it.
cv::Mat Camera::getFrameGrayscale(bool use_prev_held_frame)
{
	cv::Mat return_frame;
	
	
	if (!use_prev_held_frame)
	{
		if ((!frame_raw.data == NULL) && (frame_raw.depth() != 1))
			cvtColor(frame_raw, return_frame, CV_BGR2GRAY);
	}
	else
	{
		if ((!held_frame.data == NULL) && (held_frame.depth() != 1))
			cvtColor(held_frame, return_frame, CV_BGR2GRAY);
	}

	return return_frame;
}

// Attempts to set up a YoctoPuce USB Relay
bool Camera::initRelay()
{
	string errmsg;
	
	if (yRegisterHub("usb", errmsg) != YAPI_SUCCESS)
	{
		std::cout << "Reg error: " << errmsg << std::endl;
		return false;
	}
	this->relay = yFirstRelay();
	if (this->relay == NULL)
	{
		std::cout << "couldn't find a relay" << std::endl;
		return false;
	}

	this->relay_state = this->relay->get_state();
	relay_init = true;
	leds = false;

	return true;
}

bool Camera::switchRelay()
{
	Y_STATE_enum new_state;

	if (relay_state == Y_STATE_enum::Y_STATE_A)
		new_state = Y_STATE_enum::Y_STATE_B;
	else
		new_state = Y_STATE_enum::Y_STATE_A;

	relay->set_state(new_state);

	if (relay->get_state() == new_state)
		return true;
	else
		return false;
}

bool Camera::setRelay(bool value)
{
	Y_STATE_enum new_state = value ? Y_STATE_enum::Y_STATE_B : Y_STATE_enum::Y_STATE_A;

	relay->set_state(new_state);

	if (relay->get_state() == new_state)
		return true;
	else
		return false;
}