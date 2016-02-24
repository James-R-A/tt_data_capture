#include "IPUtils.h"

// Gets the exponential transform of an image. Input should be grayscale.
// Non-grayscale input will be converted and used anyway.
// The exponential transform essentialy increases the contrast of brighter regions.
cv::Mat IPUtils::getExponential(cv::Mat image_in, int expConst, int expMult)
{
	cv::Mat input_image;
	input_image = image_in;
	cv::Mat grayscale_image;
	cv::Size im_size = image_in.size();

	// Variables for operation.
	int constant = expConst;
	int alpha_mult = expMult;
	double alpha = 0.001;

	// Check image in is grayscale, if not, change it.
	if (input_image.channels() > 1)
		cvtColor(input_image, grayscale_image, CV_BGR2GRAY);
	else
		grayscale_image = input_image;

	cv::Mat temp;
	grayscale_image.convertTo(temp, CV_64F);
	cv::Mat temp_1(im_size, CV_64F);
	// Do the exponential transformation
	// output[i,j] = constant * (b^input[i,j] - 1)
	// where b = 1 + (alpha * alpha_mult) 
	double b = 1.0 + (alpha * alpha_mult);
	for (int row = 0; row < temp.rows; row++)
	{
		double* pixel = temp.ptr<double>(row);
		for (int col = 0; col < temp.cols; col++)
		{
			temp_1.at<double>(row, col) = (pow(b, *pixel) - 1.0);
			pixel++;
		}
	}
	temp_1 = constant * temp_1;

	// Convert back to an 8 bit image and normalise across range.
	convertScaleAbs(temp_1, temp_1);
	cv::Mat output_image;
	normalize(temp_1, output_image, 0, 255, cv::NORM_MINMAX);

	return output_image;
}

// Gets the logarithmic transform of an image. Input should be grayscale.
// Non-grayscale will be converted and used anyway.
// The logarithmic transform essentially reduces the contrast of brighter regions.
cv::Mat IPUtils::getLogarithmic(cv::Mat image_in, int logConst, int logMult)
{
	cv::Mat input_image;
	input_image = image_in;
	cv::Mat grayscale_image;


	// variables for operation.
	int constant = logConst;
	int omega_mult = logMult;
	double omega = 0.01;

	// Check image in is grayscale, if not, change it.
	if (input_image.channels() > 1)
		cvtColor(input_image, grayscale_image, CV_BGR2GRAY);
	else
		grayscale_image = input_image;

	cv::Mat temp;
	grayscale_image.convertTo(temp, CV_32F);
	// Do the logarithmic transformation
	// output[i,j] = constant * log(1 + b|input[i,j]|)
	// where b = exp(omega * omega_mult) - 1
	
	double b = exp(omega*omega_mult) - 1.0;
	temp = 1 + (b * temp);
	cv::log(temp, temp);
	temp = constant * temp;

	// Convert back to an 8 bit image and normalise across range.
	convertScaleAbs(temp, temp);
	cv::Mat output_image;
	normalize(temp, output_image, 0, 255, cv::NORM_MINMAX);

	return output_image;
}

cv::Mat IPUtils::getThresholded(cv::Mat image_in, int threshold_value, int threshold_type)
{
	cv::Mat grayscale_image;

	// Check image in is grayscale, if not, change it.
	if (image_in.channels() > 1)
		cvtColor(image_in, grayscale_image, CV_BGR2GRAY);
	else
		grayscale_image = image_in;

	cv::Mat output_image = grayscale_image;
	threshold(grayscale_image, output_image, threshold_value, 255, threshold_type);

	return output_image;
}

cv::Mat IPUtils::getBilateralFiltered(cv::Mat image_in, int value)
{
	cv::Mat grayscale_image;

	// Check image in is grayscale, if not, change it.
	if (image_in.channels() > 1)
		cvtColor(image_in, grayscale_image, CV_BGR2GRAY);
	else
		grayscale_image = image_in;

	cv::Mat output_image;
	bilateralFilter(grayscale_image, output_image, 5, value, value);

	return output_image;
}

string IPUtils::getTypeString(int type) {
	string r;

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

cv::Mat IPUtils::preProcess(cv::Mat image_in, int bilat_param, int threshold_value, int threshold_type)
{
	cv::Mat temp;

	temp = IPUtils::getBilateralFiltered(image_in, bilat_param);
	return IPUtils::getThresholded(temp, threshold_value, threshold_type);
}

std::vector<int> IPUtils::generateDepthBinMap(bool zero_bin, int total_bins, int max)
{
    std::vector<int> binMap(max+1);
    int ranged_classes = zero_bin ? total_bins - 1 : total_bins;
    int division = (int)ceil((float)max / ranged_classes);
    int class_no = zero_bin ? 1 : 0;
    int d = 1;
    binMap[0] = 0;
    for (int i = 1; i <= max; i++)
    {
        binMap[i] = class_no;
        if (d++ == division)
        {
            class_no++;
            d = 1;
        }
    }
    return binMap;
}

cv::Mat IPUtils::getPatch(cv::Mat image, cv::Point center, int patch_size)
{
	int max_offset = (patch_size - 1) / 2;
	cv::Point patch_lb((center.x - max_offset), (center.y - max_offset));
	cv::Point patch_ub((center.x + max_offset), (center.y + max_offset));
	cv::Rect boundary(cv::Point(0, 0), image.size());
	cv::Point point_check;
	cv::Mat ret_patch = cv::Mat::zeros(patch_size, patch_size, CV_8UC1);

	int pr = 0, pc = 0;
	for (int r = patch_lb.y; r <= patch_ub.y; r++)
	{
		pc = 0;
		point_check = cv::Point(0, r);
		if (point_check.inside(boundary))
		{
			uchar* im_pix = image.ptr<uchar>(r);
			uchar* patch_pix = ret_patch.ptr<uchar>(pr);

			for (int c = patch_lb.x; c <= patch_ub.x; c++)
			{
				point_check = cv::Point(c, r);
				if (point_check.inside(boundary))
				{
					patch_pix[pc] = im_pix[c];
					pc++;
				}
				else
				{
					pc++;
					continue;
				}
			}

			pr++;
		}
		else
		{
			pr++;
			continue;
		}
	}

	return ret_patch;
}

std::vector<uchar> IPUtils::vectorFromBins(cv::Mat bin_mat, cv::Size expected_size)
{
	int samples = bin_mat.size().height;
	int bins = bin_mat.size().width;
	if (samples != expected_size.area())
		throw "Invalid sizes!";

	std::vector<uchar> out_vec;
	out_vec.resize(samples);
	int row_max;
	int row_max_index;
	for (int i = 0; i < samples; i++)
	{
		//int* bin = bin_mat.ptr<int>(i);
		row_max = bin_mat.at<int>(i,0);
		row_max_index = 0;
		for (int j = 1; j < bins; j++)
		{
			if (row_max < bin_mat.at<int>(i, j))
			{
				row_max = bin_mat.at<int>(i, j);
				row_max_index = j;
			}
		}
		out_vec[i] = uchar(row_max_index);
	}
	
	return out_vec;
}

IPUtils::IPUtils()
{
}

IPUtils::~IPUtils()
{
}
