#include "Calibration.h"

Calibration::Calibration()
{

}

Calibration::~Calibration()
{
}

void Calibration::initCalibration(int x_dims, int y_dims, float square_size, int target_type, cv::Size image_size, string calib_name, int min_req_samples)
{
	this->target_type = target_type;
	this->image_size = image_size;
	this->calib_name = calib_name;
	this->min_req_samples = min_req_samples;
	tg_x = x_dims;
	tg_y = y_dims;
	tg_n = x_dims * y_dims;
	tg_square_size = square_size;
	samples = 0;
	calibration_initialised = true;
	calibration_completed = false;
}

bool Calibration::addCalibrationImage(cv::Mat image)
{
	bool ret_bool = false;
	cv::Mat image_in = image;
	cv::Mat image_gray;
	// Vector of 2d corner coords
	vector<cv::Point2f> corners;
	cv::Size tg_size = cv::Size(tg_x, tg_y);

	// Check calibration is in progress & image is correct size
	cv::Size input_size = image_in.size();
	if ((!calibration_initialised)||(input_size.width != this->image_size.width)||(input_size.height != this->image_size.height))
		return false;

	// Use grayscale image for finding chessboard corners
	cvtColor(image_in, image_gray, CV_BGR2GRAY);
	bool found_corners = false;
	if (target_type == TARGET_CHESSBOARD)
	{
		found_corners = findChessboardCorners(image_gray, tg_size, corners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
	}
	else if (target_type == TARGET_CIRCLES)
	{
		found_corners = findCirclesGrid(image_gray, tg_size, corners,
			cv::CALIB_CB_SYMMETRIC_GRID);
	}

	if (found_corners)
	{
		// For further accuracy, parameters taken from OPENCV calibration docs 
		cornerSubPix(image_gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));

		// Doesn't check for square chessboards. Add in later?
		// Draw located corners on image
		drawChessboardCorners(image_in, tg_size, corners, true);

		// Add the corner points to the calibration points array
		image_points_vec.push_back(corners);
		
		// Save the image for calibration
		//string filename = "calib_image_";
		//filename = filename + to_string(samples) + ".png";
		//imwrite(filename, image_in);
		// Increment the number of calibration samples
		samples++;
		ret_bool = true;
	}

	return ret_bool;
}

bool Calibration::performCalibration()
{
	bool ret_bool = false;
	
	cv::Mat camera_matrix;
	cv::Mat dist_coeffs;
	vector<cv::Mat> rot_mats;
	vector<cv::Mat> trans_mats;
	
	if (calibration_initialised && (!calibration_completed) && (samples >= min_req_samples))
	{
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
							ofset = tg_square_size / 2.0;
						else
							ofset = 0.0;

						tg_points_vec[i][j*tg_x + k] = cv::Point3f((k * tg_square_size) + ofset, j * (tg_square_size / 2), 0);
					}
					odd = !odd;
				}
			}
		}
		// Calibrate the camera:
		// INPUTS: tg_points - vector of vector of chessboard corner positions
		//		   image_points_vec - vector of vector of chessboard corner position 
		//                            found in samples
		//		   image_size - the size of the image samples
		//		   flags - no clue whatsoever which to chose, so none atm.
		//         Term - the terms for finishing the optimisation. Again, no clue
		//                guessed at 100 iterations or change < 0.001
		// OUTPUTS:camera_matrix - matrix of the camera intrinsics
		//         dist_coeffs - matrix of distortion coefficients
		//         rot_mats - vector of rotation matrices
		//         trans_mats - vector of translation matrices
		int flags = CV_CALIB_ZERO_TANGENT_DIST;
		calibrateCamera(tg_points_vec, image_points_vec, image_size,
			camera_matrix, dist_coeffs,
			rot_mats, trans_mats, flags,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.001));

		// empty matrix R defaults to identity transform. DON'T use a matrix of 0's
		cv::Mat R;
		cv::Mat new_cam_mat = getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 1, image_size);
		// Create the maps for undistortion. Used in remap_image()
		initUndistortRectifyMap(camera_matrix, dist_coeffs, R, new_cam_mat, image_size, CV_32FC1, map1, map2);

		calibration_initialised = false;
		calibration_completed = true;

		ret_bool = true;
	}
	else
		ret_bool = false;

	return ret_bool;
}

cv::Mat Calibration::remapImage(cv::Mat input_image)
{
	cv::Mat output_image;

	// If a calibration has been completed, use it to remap an image.
	if (calibration_completed)
	{
		remap(input_image, output_image, map1, map2, cv::INTER_LINEAR);
	}
	else
		output_image = input_image;
	
	return output_image;
}
