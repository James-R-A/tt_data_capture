#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <algorithm>
#include <Windows.h>

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <pxccapture.h>

// C:\Users\James\workspace\YoctoLib.cpp.22835\Binaries\windows\amd64
// C:\Users\James\workspace\YoctoLib.cpp.22835\Binaries\windows\yapi\amd64

#include "Calibration.h"
#include "ParallelCapture.h"
#include "IPUtils.h"

using namespace std;

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define CAM_CHECK 5
#define CALIB_FRAME_GAP 10
#define CHESSBOARD_XDIM 9
#define CHESSBOARD_YDIM 6
#define CHESSBOARD_SQ_DIMENSION 40.0 //in mm
#define AC_XDIM 4
#define AC_YDIM 11
#define AC_GAP 20.5
// 0 for chessboard, 1 for asymmetric circles
#define TARGET 0
#define LOOP_DELAY 30
#define MIN_CALIB_SAMPLES 25
#define SAVE_PATH "D:\\training_realsense\\img"

int cvCameraStreams()
{
	int i;
	int h;
	bool continue_flag = true;
	// Check which cameras open successfully from zero to cam_check.
	vector<int> cam_list = Camera::detectCameras(CAM_CHECK, false);
	int cam_list_size = cam_list.size();
	string temp_name = "";
	vector<string> cam_names;

	while (continue_flag)
	{
		// Display available cameras.
		std::cout << "Available Cameras:\n" << endl;
		for (i = 0; i < cam_list_size; i++)
		{
			std::cout << "\t" << to_string(cam_list[i]) << endl;
			temp_name = "camera_" + to_string(cam_list[i]);
			cam_names.push_back(temp_name);
		}

		// If no cameras could be opened, quit.
		if (cam_list_size == 0)
		{
			std::cout << "Failed to open any cameras.\n Exiting" << endl;
			std::cin >> h;
			return -1;
		}

		std::cout << "\nTo view the output of a camera, enter the camera number:" << endl;
		std::cin >> h;
		if (std::binary_search(cam_list.begin(), cam_list.end(), h))
		{
			string name = "camera_" + to_string(h);
			Camera cam;
			cam.setup(h, name, cv::Size(IMG_WIDTH, IMG_HEIGHT));
			bool cont = true;
			if (cam.open)
			{
				cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
				while (cont)
				{
					cam.doCapture();
					cv::imshow(name, cam.frame_raw);
					if (cv::waitKey(30) >= 0)
						cont = false;
				}
			}
			cv::destroyAllWindows();
		}
		else
			continue_flag = false;
		// Refresh cin buffer
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}

	cv::destroyAllWindows();
	return 0;
}

vector<int> calibMenu(vector<int> cam_vec)
{
	bool rpt = true;
	int cam_vec_size = cam_vec.size();
	vector<int> ret_vec;
	vector<vector<int>> cal_cam_vec(cam_vec_size);
	int sel;
	string tmp;

	// Compile vector of pairs, camera number and 0 if not selected
	// for calibration, 1 if selected
	for (int i = 0; i < cam_vec_size; i++)
	{
		cal_cam_vec[i] = { cam_vec[i], 0 };
	}

	while (rpt)
	{
		std::cout << "\nTo add a camera to the calibration list, input its camera number" << endl;
		std::cout << "To remove a cmaera from the calibration list, input its camera number" << endl;
		std::cout << "To begin calibration, input 99" << endl;

		// Display available cameras.
		std::cout << "\n Available Cameras:";
		std::cout << "\t| Selected for Calibration?" << endl;
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~|~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		for (int i = 0; i < cam_vec_size; i++)
		{
			std::cout << "\t\t" << to_string(cal_cam_vec[i][0]);
			// If the camera is set to be calibrated or not
			tmp = (cal_cam_vec[i][1] == 1) ? "Yes" : "No";
			std::cout << "\t|\t" << tmp << endl;
		}

		std::cin >> sel;
		if (sel != 99)
		{
			for (int i = 0; i < cam_vec_size; i++)
			{
				// If input == a camera number, flip the calibrate flag
				if (sel == i)
					cal_cam_vec[i][1] = (cal_cam_vec[i][1] == 0) ? 1 : 0;
			}
			// Clear input buffer
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
		else
			rpt = false;
	}

	// Construct vector of cameras selected for calibration.
	for (int i = 0; i < cam_vec_size; i++)
	{
		if (cal_cam_vec[i][1] == 1)
			ret_vec.push_back(cal_cam_vec[i][0]);
	}

	// return the vector of camera numbers selected for calibration
	return ret_vec;
}

int cvCameraCalib()
{
	bool continue_flag = true;
	int h;
	// Check which cameras open successfully from zero to cam_check.
	vector<int> cam_vec = Camera::detectCameras(CAM_CHECK, false);

	std::cout << "*************************Calibration Function*************************" << endl;

	// Get the user to select which cameras they want to calibrate
	vector<int> cal_vec = calibMenu(cam_vec);
	int size_cal_vec = cal_vec.size();

	// Setup vector of camera names
	vector<string> cam_names(size_cal_vec);
	// If there are cameras selected for calib, display them and add their name to vector
	std::cout << "\nCameras selected for calibration:" << endl;
	if (size_cal_vec > 0)
	{
		for (int i = 0; i < size_cal_vec; i++)
		{
			std::cout << to_string(cal_vec[i]) << endl;
			cam_names[i] = "camera_" + to_string(cal_vec[i]);
		}
	}
	else
	{
		// No point continuing if there's no working cameras.
		std::cout << "No cameras selected for calibration." << endl;
		return -1;
	}

	// Initialise and setup camera objects for each available camera for calibration.
	vector<Camera> camera(size_cal_vec);
	for (int i = 0; i < size_cal_vec; i++)
	{
		// This step is necessary as, for some reason, trying to open all 3 RealSense 
		// streams through openCV doesn't work and doesn't get caught in the detect
		// cameras...
		if (camera[i].setup(cal_vec[i], cam_names[i], cv::Size(IMG_WIDTH, IMG_HEIGHT)))
			std::cout << "Successfully opened " << camera[i].camera_name << endl;
		else
		{
			std::cout << "Failed to open " << camera[i].camera_name;
			std::cout << "\nRemoving from calibration" << endl;
			camera.erase(camera.begin() + i);
			size_cal_vec--;
			i--;
		}
	}
	std::cout << endl;
	int number_cameras = size_cal_vec;

	// Create and initialise a calibration object for each "active" camera.
	vector<Calibration> calib(number_cameras);
	for (int i = 0; i < number_cameras; i++)
	{
		calib[i].initCalibration(CHESSBOARD_XDIM, CHESSBOARD_YDIM,
			CHESSBOARD_SQ_DIMENSION, TARGET_CHESSBOARD,
			cv::Size(IMG_WIDTH, IMG_HEIGHT), camera[i].camera_name, MIN_CALIB_SAMPLES);
		if (calib[i].calibration_initialised)
			std::cout << "Calibration initialised for " << camera[i].camera_name << endl;
		else
		{
			std::cout << "Calibration initialisation failed, removing ";
			std::cout << camera[i].camera_name << endl;
			camera.erase(camera.begin() + i);
			calib.erase(calib.begin() + i);
			i--;
			number_cameras--;
		}
	}

	// Make some windows to display feeds
	for (int i = 0; i < number_cameras; i++)
	{
		cv::namedWindow(camera[i].camera_name, CV_WINDOW_AUTOSIZE);
	}
	vector<int> frame_count(number_cameras, 0);
	cv::Mat show_frame;
	while (continue_flag)
	{
		int64 start_time = cv::getTickCount();
		string print_string;
		// In separate for-loop to try and ensure frames 
		// are captured as close to concurrency as possible.
		for (int i = 0; i < number_cameras; i++)
		{
			camera[i].doCapture();
		}
		// For each camera, if there's been enough frames since the last calibration
		// image, try and add a calibration image.
		// Reset or increment the appropriate frame counter.
		for (int i = 0; i < number_cameras; i++)
		{
			show_frame = camera[i].frame_raw;

			// If there's no calibration for the camera, try and add a calibration sample
			if ((!calib[i].calibration_completed) && (calib[i].calibration_initialised))
			{
				if (frame_count[i] >= CALIB_FRAME_GAP)
				{
					calib[i].addCalibrationImage(show_frame);
					frame_count[i] = 0;
				}
				else
					frame_count[i]++;

				print_string = "Frames captured: " + to_string(calib[i].samples) + "/" + to_string(MIN_CALIB_SAMPLES);
			}
			else if (calib[i].calibration_completed)
			{
				show_frame = calib[i].remapImage(show_frame);
				print_string = "Calibration performed with " + to_string(calib[i].samples) + " samples";
			}
			else
			{
				print_string = "No calibration initialised or completed";
			}

			// overlay text onto frame.
			putText(show_frame, print_string, cv::Point(15, 15), 4, 0.5, cv::Scalar(0, 255, 0), 1, 8);
			cv::imshow(camera[i].camera_name, show_frame);
		}
		int64 process_time = (((cv::getTickCount() - start_time) / cv::getTickFrequency()) * 1000);
		int wait_time = std::max(2, (int)(LOOP_DELAY - process_time));
		h = cv::waitKey(wait_time);
		if (h == 108)
		{
			for (int i = 0; i < number_cameras; i++)
			{
				if (calib[i].samples >= calib[i].min_req_samples)
				{
					std::cout << "Attempting to calibrate " << camera[i].camera_name << endl;
					if (calib[i].performCalibration())
						std::cout << "\nCalibration successful" << endl;
					else
						std::cout << "\nCalibration failed" << endl;
				}
				else
					std::cout << "Not enough samples for " << camera[i].camera_name << endl;
			}
		}
		else if (h >= 0)
			continue_flag = false;
	}

	return 0;
}

int cvPairCalib()
{
	RealSense depth_cam;
	Camera cam;
	string cam_name;
	string dcam_name = "depth";
	int cam_no = 0;
	int input;
	bool continue_flag = true;
	bool cap = false;
	bool dcap = false;

	// Calibrate a webcam to the infrared camera of the RealSense Depth Cam
	std::cout << "\nPair Calibrate Function" << endl;
	std::cout << "This is basically a stereo calibration\n" << endl;
	std::cout << "Please input camera number as detected using main menu option 1" << endl;
	std::cout << "To return to main menu, enter \"99\"" << endl;
	std::cin >> input;

	if (input == 99)
		return 0;
	else if (input >= 0)
		cam_no = input;
	else
		return -1;

	input = -1;
	cam_name = "camera_" + to_string(cam_no);
	// Either realsense camera couldn't be found or something went wrong.
	if (depth_cam.setup(false, true, false, true, true))
		std::cout << "Depth cam setup successful" << endl;
	else
	{
		std::cout << "Depth cam setup failed.\nMake sure it's plugged in and everything" << endl;
		return -1;
	}

	bool cam_setup = cam.setup(cam_no, cam_name, cv::Size(IMG_WIDTH, IMG_HEIGHT));
	bool cam_relay = cam.initRelay();
	if (cam_setup&&cam_relay)
		std::cout << "Webcam setup successful" << endl;
	else
	{
		std::cout << "Webcam setup failed" << endl;
		return -1;
	}

	// Setup parallel capture arrangement.
	ParallelCapture pll;
	int frame_count = 0;
	cv::Mat disp_image_cam;
	cv::Mat disp_image_depth;

	cv::namedWindow(cam_name, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(dcam_name, cv::WINDOW_AUTOSIZE);
	string print_string;
	int64 start_time, strt;
	int64 process_time;
	int wait_time;

	while (continue_flag)
	{
		start_time = cv::getTickCount();

		cap = cam.doCapture();
		dcap = depth_cam.doCapture();
		
		// Grab an image frame
		if (dcap && cap)
		{

			disp_image_cam = cam.frame_raw;
			disp_image_depth = depth_cam.mat_ir;
			cv::Mat tmp;

			if (pll.calibration_completed)
			{
				print_string = "Calibration Completed with " + to_string(pll.samples) + "/" + to_string(pll.min_req_samples) + " sample images";
				if (pll.remapImages(disp_image_cam, disp_image_depth, tmp))
				{
					disp_image_cam = pll.cam_remapped;
					disp_image_depth = pll.ir_remapped;
				}
			}
			else if (pll.calibration_initialised)
			{
				frame_count++;
				print_string = "Calibration Initialised " + to_string(pll.samples) + "/" + to_string(pll.min_req_samples) + " sample images";
				if (frame_count >= CALIB_FRAME_GAP)
				{
					// Sometimes sticks here
					pll.addCalibrationImage(disp_image_cam, disp_image_depth);
					frame_count = 0;
				}
			}
			else
				print_string = "Camera feed";

			putText(disp_image_cam, print_string, cv::Point(15, 15), 4, 0.5, cv::Scalar(0, 255, 0), 1, 8);
			putText(disp_image_depth, print_string, cv::Point(15, 15), 4, 0.5, cv::Scalar(0, 255, 0), 1, 8);
			cv::imshow(cam_name, disp_image_cam);
			cv::imshow(dcam_name, disp_image_depth);
		}
		else
			std::cout << "Capture failed" << endl;
		// opencv requires a wait time of at least 30ms between imshow calls.
		// compute the minimum possible wait time to improve framerate.
		process_time = (((cv::getTickCount() - start_time) / cv::getTickFrequency()) * 1000);
		wait_time = std::max(2, (int)(LOOP_DELAY - process_time));
		input = cv::waitKey(wait_time);

		if (input == 99)
		{
			if (!pll.calibration_initialised && !pll.calibration_completed)
			{
				if (TARGET == 1)
				{
					pll.initCalibration(AC_XDIM, AC_YDIM, AC_GAP,
						TARGET_ASYMMETRIC_CIRCLES, cv::Size(IMG_WIDTH, IMG_HEIGHT), MIN_CALIB_SAMPLES);
				}
				else
				{
					pll.initCalibration(CHESSBOARD_XDIM, CHESSBOARD_YDIM, CHESSBOARD_SQ_DIMENSION,
						TARGET_CHESSBOARD, cv::Size(IMG_WIDTH, IMG_HEIGHT), MIN_CALIB_SAMPLES);
				}
			}
			else if (pll.calibration_initialised && !pll.calibration_completed)
			{
				std::cout << "Attempting calibration" << endl;
				bool success = pll.performCalibration();
				if (success)
					std::cout << "Calibration Successful" << endl;
				else
					std::cout << "Calibration Failed" << endl;
			}
		}
		else if (input >= 0)
		{
			std::cout << input << endl;
			continue_flag = false;
		}
	}

	cv::destroyAllWindows();
	return 0;
}

cv::Mat getBinned(std::vector<int>& lut, cv::Mat depth_image)
{
	cv::Mat binned(480, 640, CV_8UC1);
	cv::Mat binned_norm(480, 640, CV_8UC1);
	int rows = depth_image.size().height;
	int cols = depth_image.size().width;

	for (int r = 0; r < rows; r++)
	{
		uchar* binned_pix = binned.ptr<uchar>(r);
		uint16_t* depth_pixel = depth_image.ptr<uint16_t>(r);
		for (int c = 0; c < cols; c++)
		{
			if (depth_pixel[c] <= 1200)
			{
				binned_pix[c] = lut[depth_pixel[c]];
			}
			else
			{
				binned_pix[c] = 0;
			}
		}
	}
	cv::normalize(binned, binned_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	return binned_norm;
}

int realSenseFeed()
{
	bool continue_flag = true;
	int ret = -1;
	int j;
	bool colour = false;

	std::vector<int> lut = IPUtils::generateDepthBinMap(true, 5, 1200);
	for (int i = 0; i < lut.size(); i++)
	{
		std:cout << to_string(i) << " " << std::to_string(lut[i]) << std::endl;
	}

	// Initialise and set up RealSense Depth Cam
	RealSense depth_cam1;
	bool capture_success;
	bool setup_flag = depth_cam1.setup(true, true, colour, false);

	// Fail if setup failed - ie no RealSense camera could be found.
	if (!setup_flag)
	{
		std::cout << "Setup failed" << endl;
		continue_flag = false;
		return -1;
	}

	// Initialise a window for each stream.
	cv::namedWindow("ir");
	if(colour)
		cv::namedWindow("colour");
	cv::namedWindow("depth");
	// temp
	cv::namedWindow("depth_binned");
	cv::Mat db;
	cv::Mat depth_norm;

	char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
	char* trackbar_value = "Value";
	char* trackbar_filter = "Bilateral filter value";
	
	int threshold_type = 0;
	int threshold_value = 0;
	int bilat = 30;
	int delay = 10;

	cv::createTrackbar(trackbar_type, "ir", &threshold_type, 4);
	cv::createTrackbar(trackbar_value, "ir", &threshold_value, 255);
	cv::createTrackbar(trackbar_filter, "ir", &bilat, 150);
	cv::createTrackbar("capture interval", "ir", &delay, 10);
	
	cv::Mat temp;
	int f = 0;
	int frames_captured = 0;

	string text_string;
	bool first = true;
	bool recording = false;
	// Cycle through until user input, displaying each stream.
	while (continue_flag)
	{
		// Grab a frame from each feed, if successful, display the frame.
		capture_success = depth_cam1.doCapture();
		if (capture_success)
		{
			db = getBinned(lut, depth_cam1.mat_depth);
			cv::normalize(depth_cam1.mat_depth, depth_norm, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
			//temp = depth_cam1.mat_ir;
			//temp = IPUtils::getBilateralFiltered(temp, bilat);
			//temp = IPUtils::getThresholded(temp, threshold_value, threshold_type);
			temp = depth_cam1.mat_ir;
			text_string = recording ? "Recording:" : "Not Recording: ";
			text_string = text_string + "  Frames Captured: ";
			text_string = text_string + to_string(frames_captured);
			cv::putText(temp, text_string, cv::Point(15, 15), 4, 0.5, cv::Scalar(0, 255, 0), 1, 8);
			cv::imshow("ir", depth_cam1.mat_ir);
			if(colour)
				cv::imshow("colour", depth_cam1.mat_rgb);
			
			cv::imshow("depth_binned", db);
			cv::imshow("depth", depth_norm);

			if (first)
			{
				cout << "depth:" << endl;
				cout << IPUtils::getTypeString(depth_cam1.mat_depth.type()) << endl;
				if (colour)
				{
					cout << "colour:" << endl;
					cout << IPUtils::getTypeString(depth_cam1.mat_rgb.type()) << endl;
				}
				cout << "ir:" << endl;
				cout << IPUtils::getTypeString(depth_cam1.mat_ir.type()) << endl;
				first = false;
			}
			if (recording)
			{
				if (f > delay)
				{
					if (depth_cam1.saveFrame(SAVE_PATH + to_string(frames_captured+186)))
					{
						frames_captured++;
						f = 0;
					}
				}
				f++;
			}
		}
		capture_success = false;
		j = cv::waitKey(30);
		if (j == 108) //l
		{
			depth_cam1.switchLaser();
		}
		else if (j == 99) //c
		{
			recording = !recording;
		}
		else if (j >= 0)
		{
			ret = 0;
			continue_flag = false;
			std::cout << j << endl;
		}
	}
	cv::destroyAllWindows();
	return ret;
}

int trialPreProc()
{
	bool continue_flag = true;

	RealSense realsense;
	Camera webcam;

	int cam_number;
	std::cout << "Please enter webcam cam number as found in menu option 1" << std::endl;
	std::cin >> cam_number;

	bool rs_setup = realsense.setup(true, true, false, true, true);
	if (!rs_setup)
	{
		std::cout << "Realsense setup failed" << endl;
		return -1;
	}

	bool cam_setup = webcam.setup(cam_number, "Webcam", cv::Size(480, 640));
	webcam.initRelay();
	if (!(cam_setup&&webcam.relay_init))
	{
		std::cout << "webcam setup failed" << endl;
		return -1;
	}

	// Initialise a window for each stream.
	cv::namedWindow("webcam");
	cv::namedWindow("ir");
	cv::namedWindow("depth");

	char* trackbar_type = "Type";
	char* trackbar_value = "Value";
	char* trackbar_filter = "Bilateral filter value";
	char* d1 = "d1";
	char* d2 = "d2";
	char* d3 = "d3";
	char* d4 = "d4";
		
	int threshold_type = 3;
	int threshold_value = 0;
	int bilat = 30;

	int threshold_typeir = 3;
	int threshold_valueir = 0;
	int bilatir = 30;
	int delay1 = 80;
	int delay2 = 80;
	int delay3 = 80;
	int delay4 = 80;
	
	cv::createTrackbar(trackbar_type, "webcam", &threshold_type, 4);
	cv::createTrackbar(trackbar_value, "webcam", &threshold_value, 255);
	cv::createTrackbar(trackbar_filter, "webcam", &bilat, 150);

	cv::createTrackbar(trackbar_type, "ir", &threshold_typeir, 4);
	cv::createTrackbar(trackbar_value, "ir", &threshold_valueir, 255);
	cv::createTrackbar(trackbar_filter, "ir", &bilatir, 150);
	cv::createTrackbar(d1, "ir", &delay1, 500);
	cv::createTrackbar(d2, "ir", &delay2, 500);
	cv::createTrackbar(d3, "ir", &delay3, 500);
	cv::createTrackbar(d4, "ir", &delay4, 500);
	
	cv::Mat processed_image;
	cv::Mat processed_ir, processed_ir1;
	while (continue_flag)
	{
		int64 start_time = cv::getTickCount();
		realsense.setLaser(false);
		webcam.setRelay(true);
		cv::waitKey(delay1);
		webcam.doCapture();
		webcam.doCapture();
		webcam.doCapture();
		webcam.doCapture();
		webcam.doCapture();
		webcam.doCapture();
		cv::waitKey(delay2);
		realsense.setLaser(true);
		webcam.setRelay(false);
		cv::waitKey(delay3);
		realsense.doCapture();
		cv::waitKey(delay4);
		processed_image = IPUtils::preProcess(webcam.getFrameGrayscale(), bilat, threshold_value, threshold_type);
		processed_ir = IPUtils::preProcess(realsense.mat_ir, bilatir, threshold_valueir, threshold_typeir);
		processed_ir1 = IPUtils::preProcess(realsense.mat_ir1, bilatir, threshold_valueir, threshold_typeir);

		cv::imshow("ir1", realsense.mat_ir1);
		cv::imshow("webcam", processed_image);
		cv::imshow("ir", realsense.mat_ir);
		cv::imshow("depth", realsense.mat_depth);

		int64 process_time = (((cv::getTickCount() - start_time) / cv::getTickFrequency()) * 1000);
		int wait_time = std::max(2, (int)(LOOP_DELAY - process_time));
		int ret = cv::waitKey(wait_time);
		if (ret == 108)
		{
			
		}
		else if (ret >= 0)
			continue_flag = false;
	}
	cv::destroyAllWindows();
	return 0;
}

void printMenu()
{
	std::cout << "*************************Training and Test Data Capture*************************";
	std::cout << endl;
	std::cout << "Enter 1 to view all available raw streams using open cv" << endl;
	std::cout << "Enter 2 to calibrate some cameras" << endl;
	std::cout << "Enter 3 to perform a pair camera calibration" << endl;
	std::cout << "Enter 4 to view streams through RealSense SDK" << endl;
	std::cout << "Enter 5 to play with pre-processing params" << endl;
	std::cout << "Enter q to quit" << endl;
	std::cout << "\n" << endl;
}

int main()
{
	bool cont = true;
	string in;
	printMenu();

	// Poll for user input to chose program mode
	while (cont)
	{
		in == "";
		std::cin >> in;

		if (in.compare("1") == 0)
		{
			cvCameraStreams();
			printMenu();
		}
		else if (in.compare("2") == 0)
		{
			cvCameraCalib();
			printMenu();
		}
		else if (in.compare("3") == 0)
		{
			cvPairCalib();
			printMenu();
		}
		else if (in.compare("4") == 0)
		{
			realSenseFeed();
			printMenu();
		}
		else if (in.compare("5") == 0)
		{
			// pre-processing params
			trialPreProc();
			printMenu();
		}
		else if (in.compare("q") == 0)
			cont = false;

		// Refresh cin buffer
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}


}