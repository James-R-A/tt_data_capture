#include "ParallelCapture.h"
#include "IPUtils.h"

ParallelCapture::ParallelCapture()
{

}


ParallelCapture::~ParallelCapture()
{

}

void ParallelCapture::initCalibration(int x_dims, int y_dims, float square_size, int target_type, cv::Size image_size, int min_req_samples)
{
	this->target_type = target_type;
	this->image_size = image_size;
	this->min_req_samples = min_req_samples;
	tg_x = x_dims;
	tg_y = y_dims;
	tg_n = x_dims * y_dims;
	tg_square_size = square_size;
	samples = 0;
	calibration_initialised = true;
	calibration_completed = false;
}

bool ParallelCapture::addCalibrationImage(cv::Mat cam_frame, cv::Mat ir_frame)
{
	if (!calibration_initialised || calibration_completed)
		return false;
	
	bool ret_bool = false;
	//Mat ir_frame_exp = IPUtils::getExponential(ir_frame);
	//Mat ir_frame_exp = ir_frame;
	cv::Mat ir_frame_filtered;
	bilateralFilter(ir_frame, ir_frame_filtered, 3, 20, 20);
	cv::Mat image_with_corners[2] = { ir_frame_filtered, cam_frame };
	cv::Mat image[2] = { ir_frame_filtered, cam_frame };
	// Use grayscale image for finding chessboard corners
	cvtColor(image[1], image[1], CV_BGR2GRAY);
	cv::Size tg_size = cv::Size(tg_x, tg_y);

	// Vector of 2d corner coords
	vector<cv::Point2f> corners[2];
	bool found_corners[2];

	for (int i = 0; i < 2; i++)
	{
		if (target_type == TARGET_CHESSBOARD)
		{
			found_corners[i] = findChessboardCorners(image[i], tg_size, corners[i],
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
		}
		else if (target_type == TARGET_CIRCLES)
		{
			found_corners[i] = findCirclesGrid(image[i], tg_size, corners[i],
				cv::CALIB_CB_SYMMETRIC_GRID);
		}
		else if (target_type == TARGET_ASYMMETRIC_CIRCLES)
		{
			found_corners[i] = findCirclesGrid(image[i], tg_size, corners[i],
				cv::CALIB_CB_ASYMMETRIC_GRID);
		}
	}
	if (found_corners[0])
		std::cout << "rs" << std::endl;

	if (found_corners[1])
		std::cout << "cam" << std::endl;

	// If we found the chessboard in both images, refine and store the data
	if (found_corners[0] && found_corners[1])
	{
		for (int i = 0; i < 2; i++)
		{
			if (target_type == TARGET_CHESSBOARD)
			{
				// For further accuracy, parameters taken from OPENCV calibration docs 
				cornerSubPix(image[i], corners[i], cv::Size(11, 11), cv::Size(-1, -1),
					cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
			}
			// Add the corner points to the calibration points array
			image_points_vec[i].push_back(corners[i]);
			drawChessboardCorners(image_with_corners[i], tg_size, (cv::Mat)corners[i], true);
		}

		char name[40];
		sprintf(name, "D:/calib_images/calibration%02dIR.png", samples);
		imwrite(name, image_with_corners[0]);
		sprintf(name, "D:/calib_images/calibration%02dcolour.png", samples);
		imwrite(name, image_with_corners[1]);
		samples++;
		ret_bool = true;
	}

	return ret_bool;
}

bool ParallelCapture::performCalibration()
{ 
	if((!calibration_initialised) || calibration_completed)
		return false;

	bool ret_bool = false;

	if (samples >= min_req_samples)
	{
		calibration_initialised = false;
		// Compile vector of chessboard positions for each sample
		// (the top level elements should all be identical if same board used)
		tg_points_vec.resize(samples, vector<cv::Point3f>(tg_n));
		if (target_type != TARGET_ASYMMETRIC_CIRCLES)
		{
			for (int i = 0; i < samples; i++)
			{
				for (int j = 0; j < tg_y; j++)
				{
					for (int k = 0; k < tg_x; k++)
					{
						tg_points_vec[i][j*tg_x + k] = cv::Point3f(k * tg_square_size, j * tg_square_size, 0);
					}
				}
			}
		}
		else
		{
			for (int i = 0; i < samples; i++)
			{
				bool odd = false;
				float ofset = 0.0;
				for (int j = 0; j < tg_y; j++)
				{
					for (int k = 0; k < tg_x; k++)
					{
						if (odd)
							ofset = tg_square_size;
						else
							ofset = 0.0;

						tg_points_vec[i][j*tg_x + k] = cv::Point3f((2*k)+ofset, (j * tg_square_size), 0);
					}
					odd = !odd;
				}
			}
		}

		// variables for calibration
		cv::Mat cam_mat0, cam_mat1, dist_mat0, dist_mat1;
		vector<cv::Mat> rot_mat0, rot_mat1, tran_mat0, tran_mat1;

		int flags = 0;
		calibrateCamera(tg_points_vec, image_points_vec[0], image_size,
			cam_mat0, dist_mat0,
			rot_mat0, tran_mat0, flags,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.001));

		calibrateCamera(tg_points_vec, image_points_vec[1], image_size,
			cam_mat1, dist_mat1,
			rot_mat1, tran_mat1, flags,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.001));

		cv::Mat R, T, E, F;
		flags = CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_INTRINSIC;
		double temp;
		temp = stereoCalibrate(tg_points_vec, image_points_vec[0], image_points_vec[1],
			cam_mat0, dist_mat0,
			cam_mat1, dist_mat1,
			image_size, R, T, E, F, flags,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.001));

		cout << temp << endl;
		// Vairables for rectification
		cv::Mat rot0, rot1, proj0, proj1, q;
		stereoRectify(cam_mat0, dist_mat0, cam_mat1, dist_mat1, image_size,
			R, T, rot0, rot1, proj0, proj1, q, 0, -1);

		// Get remap matrices for both sides of the camera
		initUndistortRectifyMap(cam_mat0, dist_mat0, rot0, proj0, image_size, CV_32FC1, map0x, map0y);
		initUndistortRectifyMap(cam_mat1, dist_mat1, rot1, proj1, image_size, CV_32FC1, map1x, map1y);

		calibration_completed = true;
		ret_bool = true;
	}

	return ret_bool;
}

bool ParallelCapture::remapImages(cv::Mat cam_frame, cv::Mat depthcam_ir, cv::Mat depthcam_depth)
{
	bool ret_bool = false;
	if (!calibration_completed)
		return false;

	int check = 0;
	if (!cam_frame.empty())
	{
		remap(cam_frame, cam_remapped, map1x, map1y, cv::INTER_LINEAR);
		check++;
	}
	if (!depthcam_ir.empty())
	{
		remap(depthcam_ir, ir_remapped, map0x, map0y, cv::INTER_LINEAR);
		check++;
	}
	if (!depthcam_depth.empty())
	{
		remap(depthcam_depth, depth_remapped, map0x, map0y, cv::INTER_LINEAR);
		check++;
	}
	if(check>0)
		ret_bool = true;
	
	return ret_bool;
}

bool ParallelCapture::saveCalibration(string filename)
{
	if (!calibration_completed)
		return false;

	const char * filename_c = filename.c_str();
	FILE* f = fopen(filename_c, "wb");
	try
	{
		if (!f) return false;
		if (!fwrite(map0x.data, sizeof(float), map0x.rows*map0x.cols, f)) return false;
		if (!fwrite(map0y.data, sizeof(float), map0y.rows*map0y.cols, f)) return false;
		if (!fwrite(map1x.data, sizeof(float), map1x.rows*map1x.cols, f)) return false;
		if (!fwrite(map1y.data, sizeof(float), map1y.rows*map1y.cols, f)) return false;
		fclose(f);
	}
	catch (...)
	{
		fclose(f);
		throw;
	}

	return true;
}

bool ParallelCapture::loadCalibration(string path)
{
	const char * path_c = path.c_str();

	map0x = cv::Mat(image_size, CV_32FC1);
	map0y = cv::Mat(image_size, CV_32FC1);
	map1x = cv::Mat(image_size, CV_32FC1);
	map1y = cv::Mat(image_size, CV_32FC1);

	FILE* f = fopen(path_c, "rb");
	try
	{
		if (!f) return false;
		if (!fread(map0x.data, sizeof(float), map0x.rows*map0x.cols, f)) return false;
		if (!fread(map0y.data, sizeof(float), map0y.rows*map0y.cols, f)) return false;
		if (!fread(map1x.data, sizeof(float), map1x.rows*map1x.cols, f)) return false;
		if (!fread(map1y.data, sizeof(float), map1y.rows*map1y.cols, f)) return false;
		fclose(f);
	}
	catch (...)
	{
		fclose(f);
		throw;
	}

	calibration_completed = true;
	return true;
}