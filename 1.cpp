#include"line-extraction.h"
#include<iomanip>
#include<fstream>
#include<unordered_map>
#include<math.h>
using namespace std;
using namespace cv;

#define MIN_VALUE 0.00001
#define MAX_VALUE 100000

struct Pixel_data {
	int x;
	int intensity;
	double dx;
	double dxx;
};

float estimate_line_width(cv::Mat& img);

std::vector<int> find_line_region(cv::Mat& img, const int line_width, const int row_index, cv::Mat& mask);

double subpixel_x_value(cv::Mat& img, const int x, const int line_width, const int row_index);

cv::Mat ComputeDerivative(cv::Mat& img);

cv::Mat CreateMask(cv::Mat& diffImage, std::vector<int> &x1, std::vector<int> &x2);

cv::Mat GenerateMask(cv::Mat& diffImage);

cv::Mat PreProcessImage(cv::Mat& img);

void SetFilterMat(Mat & Dx, Mat & Dy, Mat & Dxx, Mat & Dyy, Mat & Dxy, int size_m);

bool isMax(int i, int j, Mat & img, int dx, int dy);

void StegerLine(Mat & img0, vector<Point2d>& sub_pixel, int size_m, int index);

void LineMatch(cv::Mat& left_img, cv::Mat& right_img, const float& baseline, const float& focal_length, std::vector<std::vector<cv::Point2d>> res);


int main()
{
	//*****
	cv::Mat img1 = cv::imread("cam1.BMP", 0);
	cv::Mat img2 = cv::imread("cam2.BMP", 0);
	cv::Mat result;
	cv::Mat source1 = img1.clone();
	cv::Mat source2 = img2.clone();
	cv::absdiff(source1, source2, result);
	cv::Rect Left = cv::Rect(0,0,result.cols/2,result.rows);
	
	cv::Rect Right = cv::Rect(result.cols/2+1, 0, result.cols-1, result.rows-1);
	//cv::Mat left_image = result(Left);
	cv::Mat right_image = result[0:1280,0:960];
	//cv::imshow("aa",left_image);
	cv::imshow("bb",right_image);
	cv::waitKey(0);
	Mat src = imread("mask.PNG",1);
	vector<Point2d>sub_pixel;
	StegerLine(src, sub_pixel, 5, 1);

	return 0;
	//*********
	//-----------------pre process
	cv::Mat Img1 = cv::imread("cam1.BMP",0);
	cv::Mat Img2 = cv::imread("cam2.BMP",0);
	cv::Mat res, res2;
	cv::Mat src1 = Img1.clone();
	cv::Mat src2 = Img2.clone();
	//0224

	
	//for color
	/*
	cv::Mat ch3_2, ch3_1,ch1_1,ch1_2;
	// "channels" is a vector of 3 Mat arrays:
	std::vector<cv::Mat> channels_1(3);
	std::vector<cv::Mat> channels_2(3);
	// split img:
	cv::split(Img1, channels_1);
	cv::split(Img2, channels_2);
	// get the R channel (dont forget they follow BGR order in OpenCV)

	ch3_1 = channels_1[2];
	ch3_2 = channels_2[2];
	// calc difference map
	cv::absdiff(ch3_1, ch3_2, res);
	*/
	cv::absdiff(src1, src2, res);
	std::vector <int> x1;
	std::vector <int> x2;
	//create Mask
	cv::Mat mask = CreateMask(res, x1, x2);
	
	cv::imwrite("Rdiff.BMP", mask);
	//today
	cv::GaussianBlur(mask, mask, cv::Size(0, 0), 3, 3);
	int linew = estimate_line_width(mask);
	cout <<"linewidth : "<< linew << endl;
	cv::Mat src_gray = mask.clone();
	cv::Mat first;
	src_gray.convertTo(src_gray, CV_32FC1);
	first.convertTo(first, CV_32FC1);
	cv::Sobel(src_gray, first, CV_32F, 1, 0, 3);
	cv::Mat second;
	second.convertTo(second, CV_32FC1);
	cv::Sobel(first, second, CV_32F, 1, 0, 3);
	
	std::ofstream point1("point.txt", ios::out);
	for (int i = 0; i < mask.rows/*Src.rows*/; i++)
	{
		std::cout << x1[i] << std::endl;
		std::cout << "i = " << i << std::endl;
		for (int j = x1[i] - linew; j < x1[i] + linew; j++)
		{
			std::cout << "1: " << first.at<float>(i, j)  << " 2: " << second.at<float>(i, j) << std::endl;
			
			if (first.at<float>(i, j) > 0 && first.at<float>(i, j + 1) < 0 /*&& abs(first.at<float>(i, j) - first.at<float>(i, j + 1)) > 100*/)
			{
				if (abs(first.at<float>(i, j + 1)/ second.at<float>(i, j + 1))<0.5)
				{
					float x1_res = j + 1 - (first.at<float>(i, j+1) / second.at<float>(i, j+1));
					std::cout << " x:  " << j + 1 - (first.at<float>(i, j+1) / second.at<float>(i, j+1)) << std::endl;
					if (point1.is_open()) {
						point1 << x1_res << ", " << i << endl;
						
					}
					else {
						cout << "cannot open filestream" << endl;
					}
				}

			}
			
		}
	}
	for (int i = 0; i < mask.rows/*Src.rows*/; i++)
	{
		std::cout << x2[i] << std::endl;
		std::cout << "i = " << i << std::endl;
		for (int j = x2[i] - linew; j < x2[i] + linew; j++)
		{
			//std::cout << "1: " << first.at<float>(i, j) << " 2: " << second.at<float>(i, j) << std::endl;

			if (first.at<float>(i, j) > 0 && first.at<float>(i, j + 1) < 0 /*&& abs((j+1)-x2[i])<=1*/)
			{
				//std::cout << "??:: " << abs(first.at<float>(i, j) / second.at<float>(i, j)) << std::endl;
				if (abs(first.at<float>(i, j+1) / second.at<float>(i, j+1)) < 0.5)
				{
					float x2_res = j + 1 - (first.at<float>(i, j + 1) / second.at<float>(i, j + 1));
					std::cout << " x:  " << j + 1 - (first.at<float>(i, j + 1) / second.at<float>(i, j + 1)) << std::endl;
					if (point1.is_open()) {
						point1 << x2_res << ", " << i << endl;
					}
					else {
						cout << "cannot open filestream" << endl;
					}
				}

			}

		}
	}
	point1.close();

	//endtoday

	//-----------------get pixel value
	Mat img0(Img1.rows, Img1.cols, CV_32FC1);
    img0 = imread("Rdiff.BMP", 0);
	int line_width = estimate_line_width(img0);
	cout << line_width << endl;
	//gaussian blur
	cv::GaussianBlur(img0, img0, cv::Size(0, 0), 3, 3);

	std::vector<int> res_x;

	cv::Mat show(img0.rows, img0.cols, CV_32FC3, Scalar(255,255, 255));
	std::ofstream point("point.txt", ios::out);
	Mat mask1(Img1.rows, Img1.cols, CV_8UC1);
	mask1 = img0.clone();
	imshow("mask", mask);
	
	for (int i = 0; i < 40/*img0.rows*/; i++)
	{
		res_x = find_line_region(img0, line_width, i, mask);
		cout <<" x value "<< res_x[0] << ", " << res_x[1] << endl << endl;
		//adjust x value for subpixel accuracy
		std::vector<double> subpixel_x;
		double x1_res = subpixel_x_value(img0, res_x[0], line_width, i);
		double x2_res = subpixel_x_value(img0, res_x[1], line_width, i);
		cout << setprecision(10) << x1_res << ", " << x2_res << endl << endl;
		if (x1_res != 0 && x2_res != 0)
		{
			cv::circle(show, cv::Point(x1_res,i), 0.5, cv::Scalar(0, 0, 255), 1, 8);
			cv::circle(show, cv::Point(x2_res, i), 0.5, cv::Scalar(0, 0, 255), 1, 8);
			
			if (point.is_open()) {
				point << x1_res << ", " << i << endl;
				point << x2_res << ", " << i << endl;
			}
			else {
				cout << "cannot open filestream" << endl;
			}
		}

	}
	point.close();
	cv::imshow("result", show);
	imwrite("mask.PNG", mask);
	imwrite("draw.PNG", show);
	cv::waitKey(0);
	/*
	cv::Mat Src = cv::imread("Rdiff.BMP", 1);
	if (!Src.data) {
		printf("fail to open the image!\n");
		return -1;
	}
	ExtractLines lines(Src);
	lines.imshow();
	

	return 0;
	*/
}

float estimate_line_width(cv::Mat& img)
{
	float res;
	cv::Mat src = img.clone();
	
	cv::threshold(src, src, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	cv::imwrite("R01.BMP", src);
	int imgcol = src.cols;
	int imgrow = src.rows;
	//imshow("??", src);
	int count = 0;
	for (int i = 0; i < imgcol; i++)
	{
		for (int j = 0; j < imgrow; j++)
		{
			if (src.at<uint8_t>(j, i) == 255)
				count++;
		}
	}
	cout << imgrow << endl;
	cout << count << endl;
	res = ((float)count / (float)(imgrow * 2));
	return res;
};

std::vector<int> find_line_region(cv::Mat& img, const int line_width, const int row_index, cv::Mat& mask)
{
	std::vector <int> x_value;
	cv::Mat src = img.clone();
	int imgcol = src.cols;
	int window_size = line_width + 2;
	int x1 = 0;
	int x2 = 0;
	int max1 = 0;
	int max2 = 0;
	for (int i = 0; i < imgcol/2; i++ )
	{
		int intensity = 0;
		
		//sliding window find the highest intensity region in the left image
		for (int j = 0; j < window_size; j++)
		{
			intensity += src.at<uint8_t>(row_index, i + j);
		}
		
		if (intensity > max1 && intensity > 150)
		{
			max1 = intensity;
			x1 = i + window_size / 2;
			
		}
		
	}
	cv::Point2d(x1, row_index);
	x_value.push_back(x1);
	for (int i = imgcol / 2; i < imgcol - window_size ; i++)
	{
		int intensity = 0;
		
		//sliding window find the highest intensity region in the left image
		for (int j = 0; j < window_size; j++)
		{
			intensity += src.at<uint8_t>(row_index, i + j);
		}
		if (intensity > max2 && intensity >150)
		{
			max2 = intensity;
			x2 = i + window_size / 2;
		}

	};
	x_value.push_back(x2);

	
	for (int i = 0; i < img.cols; i++)
	{
		if (abs(i - x1) > window_size + 1 || abs(i - x2) > window_size + 1)
		{
			int a = 0;
			mask.at<uint8_t>(row_index, i) = a;
			cout <<"my mask: value " << (int)mask.at<uint8_t>(row_index, i) << endl;
		}
	}
	
	return x_value;
}

double subpixel_x_value(cv::Mat& img, const int x, const int line_width, const int row_index)
{
	std::vector<std::vector<int>> intensity;
	std::vector<int> first_order;
	std::vector<int> second_order;
	double x_res =x;
	cv::Mat src = img.clone();

	std::vector<Pixel_data> p;
	for (int i = x - line_width / 2 - 1 ; i < x + line_width / 2 + 1 ; i++)
	{
		/*
		std::vector<int> I;
		int t = src.at<uint8_t>(row_index,i);
		//cout << t <<endl;
		I.push_back(t);
		I.push_back(x);
		intensity.push_back(I);
		*/
		Pixel_data a;
		a.intensity = src.at<uint8_t>(row_index, i);
		a.dx = (double)(src.at<uint8_t>(row_index, i + 1) - src.at<uint8_t>(row_index, i - 1)) / 2;
		a.dxx = (double)(src.at<uint8_t>(row_index, i + 1) + src.at<uint8_t>(row_index, i - 1) - 2 * src.at<uint8_t>(row_index, i));
		cout << "intensity:" << a.intensity << endl;
		cout << "x-x0" << a.dx / a.dxx << endl;
		a.x = x;
		p.push_back(a);
	}

	for (int i = 0; i < p.size(); i++)
	{
		if (p[i].dx == 0)
		{
			x_res = (double)p[i].x;
			return x_res;
		}
		else if (p[i-1].dx*p[i].dx < 0 /*&& abs(p[i].dx / p[i].dxx) < 0.5*/)
		{
			x_res = (double)p[i].x - (double)p[i].dx / (double)p[i].dxx;
			//cout << first_order[i + 1] <<", " <<second_order[i + 1]<<endl;
			cout <<"xresxres:  "<< x_res;
		}
		else
			cout <<"> 0.5 is :" <<abs((double)p[i].dx / (double)p[i].dxx) << endl;
		
		if (x_res < MIN_VALUE || x_res > MAX_VALUE)
			x_res = 0;
	}
		cout << endl;
		
	return x_res;
	/*
	for (int i = 1; i < intensity.size() - 1; i++)
	{
		first_order.push_back(intensity[i + 1][0] - intensity[i][0]);
		//cout <<" " <<(intensity[i + 1][0] - intensity[i][0]) <<"  "<< endl;
		second_order.push_back(intensity[i + 1][0] + intensity[i - 1][0] - 2 * intensity[i][0]);
		//cout << (intensity[i + 1] + intensity[i - 1] - 2 * intensity[i]) << " 2nd " << endl;
	}
	//cout << first_order.size() <<", " <<second_order.size() << endl;

	for (int i = 1; i < first_order.size()-1; i++)
	{
		//detect sign change or 1st order d
		if (first_order[i] == 0)
		{
			x_res = (double)intensity[i+1][1];
			break;
		}
		else if (first_order[i] ^ first_order[i + 1] < 0 && abs((double)first_order[i + 1] / (double)second_order[i + 1]) < 0.5)
		{
			x_res = (double)intensity[i+2][1] - (double)first_order[i + 1] / (double)second_order[i + 1];
			//cout << first_order[i + 1] <<", " <<second_order[i + 1]<<endl;
			
		}
		else
			x_res = (double)intensity[i + 3][1] - (double)first_order[i + 2] / (double)second_order[i + 2];
	}
	if (x_res < MIN_VALUE || x_res > MAX_VALUE)
		x_res = 0;

	return x_res;
	*/
	
}

cv::Mat ComputeDerivative(cv::Mat& img) 
{
	cv::Mat src = img.clone();
	//cv::Mat res(img.rows, img.cols, CV_32FC1);
	cv::Mat Mask_Dx = (cv::Mat_<float>(3, 1) << 1, 0, -1);
	cv::filter2D(src, src, CV_32FC1, Mask_Dx);
	imshow("mask", src);
	for (int i = 0; i < img.cols; i++)
	{
		for(int j = 0; j < img.rows; j++)
		cout << src.at<float>(j, i) << endl;
	}
	imwrite("b.PNG", src);
	cv::waitKey(0);

	return src;
}

cv::Mat CreateMask(cv::Mat& diffImage, std::vector<int> &x1, std::vector<int> &x2)
{
	cv::Mat mask = diffImage.clone();
	
	for (int i = 0; i < mask.rows; i++)
	{
		int max_x1 = 0;
		int current_intensity = 0;
		for (int j = 0; j < mask.cols / 2; j++)
		{
			int r_intensity;

			if (j > 2 && j < mask.cols / 2 - 2)
			{
				//r_intensity = diffImage.at<cv::Vec3b>(j, i)[2] + diffImage.at<cv::Vec3b>(j - 1, i)[2] \
					+ diffImage.at<cv::Vec3b>(j + 1, i)[2] + diffImage.at<cv::Vec3b>(j - 2, i)[2] + diffImage.at<cv::Vec3b>(j + 2, i)[2];
				r_intensity = mask.at<uint8_t>(i, j - 1) + mask.at<uint8_t>(i, j - 2) + mask.at<uint8_t>(i, j) + mask.at<uint8_t>(i, j + 1) + mask.at<uint8_t>(i, j + 2);
			}
			else
				//r_intensity = mask.at<cv::Vec3b>(j, i)[2] * 5;
				r_intensity = mask.at<uint8_t>(i, j) * 5;
			if (r_intensity > current_intensity)
			{
				max_x1 = j;
				current_intensity = r_intensity;
			}

		}
		//std::cout << max_x1 << std::endl;
		int range = max_x1;
		int max_in_range = 0;
		/*
		for (int k = range - 2; k <= range + 2; k++)
		{
			if (mask.at<uint8_t>(i, k) > max_in_range)
			{
				max_x1 = k;
				max_in_range = mask.at<uint8_t>(i, k);
			}
		}
		*/
		x1.push_back(max_x1);

	}
	for (int i = 0; i < mask.rows; i++)
	{
		for (int k = 0; k < mask.cols / 2; k++)
		{
			if (abs(k - x1[i]) > 6)
			{
				mask.at<uint8_t>(i, k) = 0;

			}
		}
	}

	for (int i = 0; i < mask.rows; i++)
	{
		int max_x2 = 0;
		int current_intensity = 0;
		for (int j = mask.cols / 2; j < mask.cols; j++)
		{
			int r_intensity;
			if (j > mask.cols / 2 + 2 && j < mask.cols - 2)
			{
				r_intensity = mask.at<uint8_t>(i, j - 1) + mask.at<uint8_t>(i, j - 2) + mask.at<uint8_t>(i, j) + mask.at<uint8_t>(i, j + 1) + mask.at<uint8_t>(i, j + 2);
			}
			else
				r_intensity = mask.at<uint8_t>(i, j) * 5;
			if (r_intensity > current_intensity)
			{
				max_x2 = j;
				current_intensity = r_intensity;
			}

		}
		x2.push_back(max_x2);
	}

	for (int i = 0; i < mask.rows; i++)
	{
		for (int k = mask.cols / 2; k < mask.cols; k++)
		{
			if (abs(k - x2[i]) > 6)
			{
				mask.at<uint8_t>(i, k) = 0;

			}
		}
	}
	
	return mask;
}

cv::Mat GenerateMask(cv::Mat& diffImage) 
{
	cv::Mat mask = diffImage.clone();
	std::vector<int> x;
	//intensity filter sliding window size 5
	for (int i = 0; i < mask.rows; i++)
	{
		int max_x = 0;
		int current_intensity = 0;
		for (int j = 0; j < mask.cols; j++)
		{
			int r_intensity;
			r_intensity = mask.at<uint8_t>(i, j + 4 ) + mask.at<uint8_t>(i, j + 3) + mask.at<uint8_t>(i, j) + mask.at<uint8_t>(i, j + 1) + mask.at<uint8_t>(i, j + 2);

			if (r_intensity > current_intensity)
			{
				max_x = j + 2;
				current_intensity = r_intensity;
			}

		}
		//std::cout << max_x1 << std::endl;
		int range = max_x;
		int max_in_range = 0;

		x.push_back(max_x);

	}
	//change useless points' intensity to 0
	for (int i = 0; i < mask.rows; i++)
	{
		for (int k = 0; k < mask.cols / 2; k++)
		{
			if (abs(k - x[i]) > 6)
			{
				mask.at<uint8_t>(i, k) = 0;

			}
		}
	}


}

struct EV_VAL_VEC {
	double nx, ny;//nx、ny为最大特征值对应的特征向量
};

void SetFilterMat(Mat& Dx, Mat& Dy, Mat&Dxx, Mat&Dyy, Mat&Dxy, int size_m)//radius * 2= size 
{
	if (size_m % 2 == 0 || size_m == 1) cout << "size is not illegal !!" << endl;
	Mat m1 = Mat::zeros(Size(size_m, size_m), CV_64F);//Dx
	Mat m2 = Mat::zeros(Size(size_m, size_m), CV_64F);//Dy
	Mat m3 = Mat::zeros(Size(size_m, size_m), CV_64F);//Dxx
	Mat m4 = Mat::zeros(Size(size_m, size_m), CV_64F);//Dyy
	Mat m5 = Mat::zeros(Size(size_m, size_m), CV_64F);//Dxy

	for (int i = 0; i < size_m / 2; i++)
	{
		m1.row(i) = 1;
		m2.col(i) = 1;
		m3.row(i) = -1;
		m4.col(i) = -1;
	}
	for (int i = size_m / 2 + 1; i < size_m; i++)
	{
		m1.row(i) = -1;
		m2.col(i) = -1;
		m3.row(i) = -1;
		m4.col(i) = -1;
	}
	m3.row(size_m / 2) = (size_m / 2) * 2;
	m4.col(size_m / 2) = (size_m / 2) * 2;
	m5.row(size_m / 2) = 1;
	m5.col(size_m / 2) = 1;
	m5.at<double>(size_m / 2, size_m / 2) = -(size_m / 2) * 4;
	if (size_m == 5) m5 = (Mat_<double>(5, 5) << 0, 0, 1, 0, 0, 0, 1, 2, 1, 0, 1, 2, -16, 2, 1, 0, 1, 2, 1, 0, 0, 0, 1, 0, 0);
	Dx = m2;
	Dy = m1;
	Dxx = m4;
	Dyy = m3;
	Dxy = m5;
	cout << Dx << endl;
	cout << Dy << endl;
	cout << Dxx << endl;
	cout << Dyy << endl;
	cout << Dxy << endl;
	return;
}

bool isMax(int i, int j, Mat& img, int dx, int dy)//在法线方向上是否为极值
{
	double val = img.at<double>(j, i);
	double max_v = max(img.at<double>(j + dy, i + dx), img.at<double>(j - dy, i - dx));
	if (val >= max_v) return true;
	else return false;
}

void StegerLine(Mat&img0, vector<Point2d>&sub_pixel, int size_m, int index)
{


	Mat img;
	cvtColor(img0, img0, CV_BGR2GRAY);
	img = img0.clone();

	//高斯滤波
	img.convertTo(img, CV_64FC1);
	GaussianBlur(img, img, Size(3, 3), 0.9, 0.9);

	//一阶偏导数
	Mat m1, m2;
	//二阶偏导数
	Mat m3, m4, m5;
	SetFilterMat(m1, m2, m3, m4, m5, size_m);

	cout << m5 << endl;

	Mat dx, dy;
	filter2D(img, dx, CV_64FC1, m1);
	filter2D(img, dy, CV_64FC1, m2);

	Mat dxx, dyy, dxy;
	filter2D(img, dxx, CV_64FC1, m3);
	filter2D(img, dyy, CV_64FC1, m4);
	filter2D(img, dxy, CV_64FC1, m5);

	//hessian矩阵
	int imgcol = img.cols;
	int imgrow = img.rows;
	vector<double> Pt;
	string filename = "point_" + std::to_string(index) + ".txt";
	ofstream points(filename, std::ios::out);
	for (int i = 0; i < imgcol - 1; i++)
	{
		for (int j = 0; j < imgrow - 1; j++)
		{
			double pixel_val = img.at<double>(j, i);
			if (img.at<double>(j, i) > 20)            //需要确定ROI！！！
			{
				Mat hessian(2, 2, CV_64FC1);
				hessian.at<double>(0, 0) = dxx.at<double>(j, i);
				hessian.at<double>(0, 1) = dxy.at<double>(j, i);
				hessian.at<double>(1, 0) = dxy.at<double>(j, i);
				hessian.at<double>(1, 1) = dyy.at<double>(j, i);

				Mat eValue;
				Mat eVectors;
				eigen(hessian, eValue, eVectors);

				double nx, ny;
				double EV_max = 0;//特征值
				double EV_min = 0;
				if (fabs(eValue.at<double>(0, 0)) >= fabs(eValue.at<double>(1, 0)))  //求特征值最大时对应的特征向量
				{
					nx = eVectors.at<double>(0, 0);//大的特征值对应的特征向量
					ny = eVectors.at<double>(0, 1);
					EV_max = max(eValue.at<double>(0, 0), eValue.at<double>(1, 0));
					EV_min = min(eValue.at<double>(0, 0), eValue.at<double>(1, 0));
				}
				else
				{
					nx = eVectors.at<double>(1, 0);//大的特征值对应的特征向量
					ny = eVectors.at<double>(1, 1);
					EV_max = max(eValue.at<double>(0, 0), eValue.at<double>(1, 0));
					EV_min = min(eValue.at<double>(0, 0), eValue.at<double>(1, 0));
				}

				//t taylor expansion
				double a = nx * nx*dxx.at<double>(j, i) + 2 * nx*ny*dxy.at<double>(j, i) + ny * ny*dyy.at<double>(j, i);
				double b = nx * dx.at<double>(j, i) + ny * dy.at<double>(j, i);
				double t = -b / a;

				int dx;
				int dy;

				if (abs(nx) >= 2 * abs(ny)) dx = 1, dy = 0;//垂直
				else if (abs(ny) >= 2 * abs(nx)) dx = 0, dy = 1;//水平
				else if (nx > 0) dx = 1, dy = 1;
				else if (ny > 0) dx = -1, dy = -1;

				if (isMax(i, j, img, dx, dy))
					//if(EV_min<-120)
				{
					double temp1 = t * nx;
					double temp2 = t * ny;
					if (fabs(t*nx) <= 0.5 && fabs(t*ny) <= 0.5)//(x + t * Nx, y + t * Ny)为亚像素坐标
					{
						Pt.push_back(i + t * nx);
						Pt.push_back(j + t * ny);
						if (!points.is_open())
						{
							std::cout << "cannot open file" << std::endl;
							break;
						}
						else
							points << i + t * nx << ", " << j + t * ny << std::endl;
					}
				}
			}
		}
	}
	std::cout << "points written to " << std::endl;
	points.close();
	for (int k = 0; k < Pt.size() / 2; k++)
	{
		Point2d rpt;
		rpt.x = Pt[2 * k + 0];
		rpt.y = Pt[2 * k + 1];
		sub_pixel.push_back(rpt);
	}
	cv::Mat Img0 = cv::imread("1.BMP", 1);
	for (unsigned int i = 0; i < sub_pixel.size(); i++) {
		cv::circle(Img0, cv::Point2d(sub_pixel[i].x, sub_pixel[i].y), 0.3, cv::Scalar(0, 200, 0));
	}
	//for debug
	cv::imshow("result", Img0);
	cv::waitKey(0);
}

cv::Mat PreProcessImage(cv::Mat& img) 
{
	cv::Mat src = img.clone();
	//create Mask
	cv::Mat mask = GenerateMask(src);
	return mask;
}

void LineMatch(cv::Mat& left_img, cv::Mat& right_img, const float& baseline, const float& focal_length, std::vector<std::vector<cv::Point2d>> res)
{
	
	std::vector<cv::Point2d> l_points;
	std::vector<cv::Point2d> r_points;
	//preprocess
	cv::Mat L = left_img.clone();
	cv::Mat R = right_img.clone();
	cv::Mat L_res = PreProcessImage(L);
	cv::Mat R_res = PreProcessImage(R);
	//steger
	StegerLine(L_res, l_points, 5, 1);
	StegerLine(R_res, r_points, 5, 2);
	
	res.push_back(l_points);
	res.push_back(r_points);

	//some operation on B and F...
	/*
	 match...
	
	
	*/
}





#include iostream
#includeiomanip
#includefstream
#includeunordered_map
#includemath.h
#include opencv2corecore.hpp
#include opencv2highguihighgui.hpp
#include opencv2imgprocimgproc.hpp
#include vector
#include string


declaration
struct EV_VAL_VEC {
	double nx, ny;nx、ny为最大特征值对应的特征向量
};

cvMat PreProcessImage(cvMat& img);

cvMat GenerateMask(cvMat& diffImage);

void SetFilterMat(cvMat& Dx, cvMat& Dy, cvMat&Dxx, cvMat&Dyy, cvMat&Dxy, int size_m);

bool isMax(int i, int j, cvMat& img, int dx, int dy);

void StegerLine(cvMat&img0, stdvectorcvPoint2d&sub_pixel, int size_m, int index);

void LineMatch(cvMat& left_img, cvMat& right_img, const float& baseline, const float& focal_length, stdvectorstdvectorcvPoint2d& res);

stdvectorcvPoint3d Img2PointCld(stdvectorstdpaircvPoint2d, cvPoint2d& matched_points, const float cameraParams);

bool SavePointCloud(const char filename, const int size, const double verts_ptr);

bool ReadCameraParameters(const char camera_file, float cameraParams);

main
int main()
{

	cvMat img1 = cvimread(cam1.BMP, 0);
	cvMat img2 = cvimread(cam2.BMP, 0);

	if (img1.empty() img2.empty())
	{
		printf( Error opening imagen);
		return EXIT_FAILURE;
	}
	diffimage
	cvMat result;
	cvMat source1 = img1.clone();
	cvMat source2 = img2.clone();
	cvabsdiff(source1, source2, result);

	crop left image and right image
	cvRect Left = cvRect(0, 0, result.cols  2, result.rows);
	cvRect Right = cvRect(result.cols  2, 0, result.cols  2, result.rows);
	cvMat left_image = result(Left);
	cvMat right_image = result(Right);

	line extract and line match
	float baseline;
	float focal_length;
	stdvectorstdvectorcvPoint2d res_1;
	LineMatch(left_image, right_image, baseline, focal_length, res_1);

	return 0;
}




functions

cvMat PreProcessImage(cvMat& img)
{
	cvMat src = img.clone();
	create Mask
	cvMat mask = GenerateMask(src);
	return mask;
}

cvMat GenerateMask(cvMat& diffImage)
{
	cvMat mask = diffImage.clone();
	stdvectorint x;
	intensity filter sliding window size 5
	for (int i = 0; i  mask.rows; i++)
	{
		int max_x = 0;
		int current_intensity = 0;
		for (int j = 0; j  mask.cols; j++)
		{
			int r_intensity;
			r_intensity = mask.atuint8_t(i, j + 4) + mask.atuint8_t(i, j + 3) + mask.atuint8_t(i, j) + mask.atuint8_t(i, j + 1) + mask.atuint8_t(i, j + 2);

			if (r_intensity  current_intensity)
			{
				max_x = j + 2;
				current_intensity = r_intensity;
			}

		}
		stdcout  max_x1  stdendl;
		int range = max_x;
		int max_in_range = 0;

		x.push_back(max_x);

	}
	change useless points' intensity to 0
	for (int i = 0; i  mask.rows; i++)
	{
		for (int k = 0; k  mask.cols; k++)
		{
			if (abs(k - x[i])  6)
			{
				mask.atuint8_t(i, k) = 0;

			}
		}
	}
	return mask;
}

void SetFilterMat(cvMat& Dx, cvMat& Dy, cvMat&Dxx, cvMat&Dyy, cvMat&Dxy, int size_m)radius  2= size 
{
	if (size_m % 2 == 0  size_m == 1) stdcout  size is illegal !!  stdendl;
	cvMat m1 = cvMatzeros(cvSize(size_m, size_m), CV_64F);Dx
	cvMat m2 = cvMatzeros(cvSize(size_m, size_m), CV_64F);Dy
	cvMat m3 = cvMatzeros(cvSize(size_m, size_m), CV_64F);Dxx
	cvMat m4 = cvMatzeros(cvSize(size_m, size_m), CV_64F);Dyy
	cvMat m5 = cvMatzeros(cvSize(size_m, size_m), CV_64F);Dxy

	for (int i = 0; i  size_m  2; i++)
	{
		m1.row(i) = 1;
		m2.col(i) = 1;
		m3.row(i) = -1;
		m4.col(i) = -1;
	}
	for (int i = size_m  2 + 1; i  size_m; i++)
	{
		m1.row(i) = -1;
		m2.col(i) = -1;
		m3.row(i) = -1;
		m4.col(i) = -1;
	}
	m3.row(size_m  2) = (size_m  2)  2;
	m4.col(size_m  2) = (size_m  2)  2;
	m5.row(size_m  2) = 1;
	m5.col(size_m  2) = 1;
	m5.atdouble(size_m  2, size_m  2) = -(size_m  2)  4;
	if (size_m == 5) m5 = (cvMat_double(5, 5)  0, 0, 1, 0, 0, 0, 1, 2, 1, 0, 1, 2, -16, 2, 1, 0, 1, 2, 1, 0, 0, 0, 1, 0, 0);
	Dx = m2;
	Dy = m1;
	Dxx = m4;
	Dyy = m3;
	Dxy = m5;
	stdcout  Dx  stdendl;
	stdcout  Dy  stdendl;
	stdcout  Dxx  stdendl;
	stdcout  Dyy  stdendl;
	stdcout  Dxy  stdendl;
	return;
}

bool isMax(int i, int j, cvMat& img, int dx, int dy)在法线方向上是否为极值
{
	double val = img.atdouble(j, i);
	double max_v = stdmax(img.atdouble(j + dy, i + dx), img.atdouble(j - dy, i - dx));
	if (val = max_v) return true;
	else return false;
}

void StegerLine(cvMat&img0, stdvectorcvPoint2d&sub_pixel, int size_m, int index)
{


	cvMat img;
	cvtColor(img0, img0, CV_BGR2GRAY);
	img = img0.clone();

	高斯滤波
	img.convertTo(img, CV_64FC1);
	GaussianBlur(img, img, cvSize(3, 3), 0.9, 0.9);

	一阶偏导数
	cvMat m1, m2;
	二阶偏导数
	cvMat m3, m4, m5;
	SetFilterMat(m1, m2, m3, m4, m5, size_m);

	stdcout  m5  stdendl;

	cvMat dx, dy;
	filter2D(img, dx, CV_64FC1, m1);
	filter2D(img, dy, CV_64FC1, m2);

	cvMat dxx, dyy, dxy;
	filter2D(img, dxx, CV_64FC1, m3);
	filter2D(img, dyy, CV_64FC1, m4);
	filter2D(img, dxy, CV_64FC1, m5);

	hessian矩阵
	int imgcol = img.cols;
	int imgrow = img.rows;
	stdvectordouble Pt;
	stdstring filename = point_ + stdto_string(index) + .txt;
	stdofstream points(filename, stdiosout);
	for (int i = 0; i  imgcol - 1; i++)
	{
		for (int j = 0; j  imgrow - 1; j++)
		{
			double pixel_val = img.atdouble(j, i);
			if (img.atdouble(j, i)  10)            需要确定ROI！！！
			{
				cvMat hessian(2, 2, CV_64FC1);
				hessian.atdouble(0, 0) = dxx.atdouble(j, i);
				hessian.atdouble(0, 1) = dxy.atdouble(j, i);
				hessian.atdouble(1, 0) = dxy.atdouble(j, i);
				hessian.atdouble(1, 1) = dyy.atdouble(j, i);

				cvMat eValue;
				cvMat eVectors;
				eigen(hessian, eValue, eVectors);

				double nx, ny;
				double EV_max = 0;特征值
				double EV_min = 0;
				if (fabs(eValue.atdouble(0, 0)) = fabs(eValue.atdouble(1, 0)))  求特征值最大时对应的特征向量
				{
					nx = eVectors.atdouble(0, 0);大的特征值对应的特征向量
					ny = eVectors.atdouble(0, 1);
					EV_max = stdmax(eValue.atdouble(0, 0), eValue.atdouble(1, 0));
					EV_min = stdmin(eValue.atdouble(0, 0), eValue.atdouble(1, 0));
				}
				else
				{
					nx = eVectors.atdouble(1, 0);大的特征值对应的特征向量
					ny = eVectors.atdouble(1, 1);
					EV_max = stdmax(eValue.atdouble(0, 0), eValue.atdouble(1, 0));
					EV_min = stdmin(eValue.atdouble(0, 0), eValue.atdouble(1, 0));
				}

				t taylor expansion
				double a = nx  nxdxx.atdouble(j, i) + 2  nxnydxy.atdouble(j, i) + ny  nydyy.atdouble(j, i);
				double b = nx  dx.atdouble(j, i) + ny  dy.atdouble(j, i);
				double t = -b  a;

				int dx;
				int dy;

				if (abs(nx) = 2  abs(ny)) dx = 1, dy = 0;垂直
				else if (abs(ny) = 2  abs(nx)) dx = 0, dy = 1;水平
				else if (nx  0) dx = 1, dy = 1;
				else if (ny  0) dx = -1, dy = -1;

				if (i  0 && j  0 && isMax(i, j, img, dx, dy))
					if(EV_min-120)
				{
					double temp1 = t  nx;
					double temp2 = t  ny;
					if (fabs(tnx) = 0.5 && fabs(tny) = 0.5)(x + t  Nx, y + t  Ny)为亚像素坐标
					{
						Pt.push_back(i + t  nx);
						Pt.push_back(j + t  ny);
						if (!points.is_open())
						{
							stdcout  cannot open file  stdendl;
							break;
						}
						else
							points  i + t  nx  ,   j + t  ny  stdendl;
					}
				}
			}
		}
	}
	stdcout  points written to   filename  stdendl;
	points.close();
	for (int k = 0; k  Pt.size()  2; k++)
	{
		cvPoint2d rpt;
		rpt.x = Pt[2  k + 0];
		rpt.y = Pt[2  k + 1];
		sub_pixel.push_back(rpt);
	}
	
	cvMat Img0 = cvimread(cam1.BMP, 1);
	for (unsigned int i = 0; i  sub_pixel.size(); i++) {
		cvcircle(Img0, cvPoint2d(sub_pixel[i].x, sub_pixel[i].y), 0.3, cvScalar(0, 200, 0));
	}
	for debug
	cvimshow(result, Img0);
	cvwaitKey(0);
	
}


void LineMatch(cvMat& left_img, cvMat& right_img, const float& baseline, const float& focal_length, stdvectorstdvectorcvPoint2d& res)
{

	stdvectorcvPoint2d l_points;
	stdvectorcvPoint2d r_points;
	preprocess
	cvMat L = left_img.clone();
	cvMat R = right_img.clone();
	cvMat L_res = PreProcessImage(L);
	cvMat R_res = PreProcessImage(R);
	cvimshow(aaa, L_res);
	cvwaitKey(0);
	cvimshow(bbb, R_res);
	cvwaitKey(0);
	steger
	StegerLine(L_res, l_points, 5, 1);
	StegerLine(R_res, r_points, 5, 2);
	cvMat L1(1280, 960, CV_32FC3);
	cvMat R1(1280, 960, CV_32FC3);
	for (unsigned int i = 0; i  l_points.size(); i++) {
		cvcircle(L1, cvPoint2d(l_points[i].x, l_points[i].y), 0.3, cvScalar(0, 200, 0));
	}

	for (unsigned int i = 0; i  r_points.size(); i++) {
		cvcircle(R1, cvPoint2d(r_points[i].x, r_points[i].y), 0.3, cvScalar(0, 200, 0));
	}
	cvimshow(aaaaa, L1);
	cvimshow(bbbbb, R1);
	cvwaitKey(0);
	res.push_back(l_points);
	res.push_back(r_points);

	some operation on B and F...
	
	
	 match...

	 disparity...
	
	
	2D-3D
	float cameraParams[9] = {1.0f};
	if (!ReadCameraParameters(camera.txt, cameraParams))
	{
		编一些值先用一下
		fx
		cameraParams[0] = 500;
		fy
		cameraParams[4] = 500;
		ox
		cameraParams[2] = 640;
		oy
		cameraParams[5] = 480;
		
		cameraParams[8] = 1;
		cameraParams[1] = 0;
		cameraParams[3] = 0;
		cameraParams[6] = 0;
		cameraParams[7] = 0;
	}
	stdvectorstdpaircvPoint2d,cvPoint2d matched_points;
	stdvectorcvPoint3d points = Img2PointCld(matched_points, cameraParams);

	save point cloud
	int num_of_points = points.size();
	double pointcloud_ptr = new double[num_of_points  3];

	memcpy(pointcloud_ptr, &points[0], sizeof(double) num_of_points 3);
	SavePointCloud(.points.obj,num_of_points, pointcloud_ptr);
	delete[] pointcloud_ptr;
}

bool SavePointCloud(const char filename, const int size, const double verts_ptr)
{
	if (verts_ptr = NULL)
		return false;
	stdstring file = filename;
	stdstring path = file.substr(0, file.rfind() + 1);
	stdstring name = file.substr(file.rfind() + 1, file.rfind(.) - file.rfind() - 1);

	stdofstream fmesh(filename);
	fmesh  # obj file generated by fengge-niubi  stdendl;
	fmesh  # Vertices   size  stdendl;
	
	for (int i = 0; i  size; i++)
	{
		fmesh  v   verts_ptr[i  3 + 0]     verts_ptr[i  3 + 1]   
			 verts_ptr[i  3 + 2];
		Red
		fmesh     1     0     0  n;
	}

	fmesh.close();
	return true;
}


stdvectorcvPoint3d Img2PointCld(stdvectorstdpaircvPoint2d, cvPoint2d& matched_points, const float cameraParams)
{
	stdvectorcvPoint3d res;

	for (int i = 0; i  matched_points.size(); i++)
	{
		cvPoint3d pt;
		float baseline = 100;
		float factor = 1;
		pt.z = baseline  cameraParams[0](matched_points[i].first.x - matched_points[i].second.x);

		pt.x = (matched_points[i].first.x - cameraParams[2]  pt.z)  cameraParams[0];
		pt.y = (matched_points[i].first.y - cameraParams[2]  pt.z)  cameraParams[4];

		res.push_back(pt);
	}

	return res;
}

bool ReadCameraParameters(const char camera_file, float cameraParams) 
{
	stdifstream fc;
	fc.open(camera_file);

	if (!fc.is_open())
	{
		stdcout  cannot read camera parameters, using mock params  stdendl;
		return false;
	}

	cameraParams[9] = { 1.0f };
	for (int i = 0; i  9; i++)
	{	
		fc  cameraParams[i];
	}
	fc.close();
}
