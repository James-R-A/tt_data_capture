#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\videoio.hpp>
#include <opencv\highgui.h>

#include <stdlib.h>
#include <stdio.h>

using namespace std;

class IPUtils
{
public:
	static cv::Mat getExponential(cv::Mat image_in, int expConst = 10, int expMult = 1);
	static cv::Mat getLogarithmic(cv::Mat image_in, int logConst = 10, int logMult = 1);
	static cv::Mat getThresholded(cv::Mat image_in, int threshold_value = 0, int threshold_type = 0);
	static cv::Mat getBilateralFiltered(cv::Mat image_in, int value=30);
	static string getTypeString(int type);

	/// <summary>
	/// Pre-process routine for infrared intensity images input to forest training
	/// or evaluation using forest. Ensure the same variables are used for each.
	/// Applies bilateral filter then a static threshold.
	/// </summary>
	/// <param name="image_in">The cv::Mat image for pre-processing.</param>
	/// <param name="bilat_param">The bilateral filter parameter (default 30),
	///                           higher values give more "cartoonish" apprearence </param>
	/// <param name="threshold_value">Value for static threshold, pixels with intensity 
	///                               below this value are set to 0 </param>
	/// <param name="threshold_type">Defaults to 3. don't change it for forest training</param>
	static cv::Mat preProcess(cv::Mat image_in, int bilat_param=30, int threshold_value=36, int threshold_type=3);

	/// <summary>
    /// Generates a pixel intensity to bin number look-up-table
    /// Bins are of even size between zero and max, with the option to include a separate zero bin
    /// </summary>
    /// <param name="zero_bin"> flag to indicate if a zero class is required </param>
    /// <param name="total_bins"> Total number of bins, including zero bin if required </param>
    /// <param name="max"> Maximum value to be included in the bins </param>
    static std::vector<int> generateDepthBinMap(bool zero_bin, int total_bins, int max);

	static cv::Mat getPatch(cv::Mat image, cv::Point center, int patch_size);

	static std::vector<uchar> vectorFromBins(cv::Mat bin_mat, cv::Size expected_size);
	IPUtils();
	~IPUtils();
};

