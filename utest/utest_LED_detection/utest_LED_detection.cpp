// utest_LED_detection.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"//因为在属性中已经配置了opencv等目录，所以把其当成了本地目录一样
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

#define HAVE_OPENCV_NONFREE

#include "filesupport/filename_support.h"
#include "filesupport/AuxFileIO.h"

void usage(bool wait = false)
{
	printf("LED detection V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("ref.ipf target.ipf trans_file --search_radius 10.0 --doICP(optional) ref_ceiling.pcd ref_wall.pcd target_ceiling.pcd target_wall.pcd --doTrans\n");
	printf("---------------------------------------------\n");
	if (wait)
	{
		printf("<press ENTER>\n");
		getc(stdin);
	}
	exit(1);
}

static void byebye(bool wait = false)
{
	if (wait)
	{
		fprintf(stderr, "<press ENTER>\n");
		getc(stdin);
	}
	/*	exit(1);*/
}

int main(int argc, char* argv[])
{
// 	std::vector<simpleCameraModel> cams;
// 	std::vector<tiePoint> tps;
// 	Load_bundleFile("D:\\test\\test_CMVS\\cmvs.nvm.cmvs\\00\\bundle.rd.out", cams, tps);


	char *pInputDir = argv[1];
	char *pOutputDir = argv[2];

	char *pExt = argv[3];

	vector <string> filenames;

#ifdef Using_SIFT
	initModule_nonfree();//if use SIFT or SURF  
	Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");
	// 	Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create("SIFT");
	// 	Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");

	if (detector.empty()/* || descriptor_extractor.empty()*/)
	{
		throw runtime_error("fail to create detector!");
		return 0;
	}

	string iFile = "";
	Mat img1 = imread(iFile/*, CV_LOAD_IMAGE_COLOR*/);

	vector<KeyPoint> keypoints1, keypoints2;
	detector->detect(img1, keypoints1);
	//	detector->detect(img2, keypoints2);
	cout << "img1:" << keypoints1.size() << " points" << endl;
	Mat img_keypoints1, img_keypoints2;
	drawKeypoints(img1, keypoints1, img_keypoints1, Scalar::all(-1), 0);
#endif

	//#ifdef Using_simpleBlob
	//simple blob detection
	SimpleBlobDetector::Params params;

	params.minThreshold = 210;
	params.maxThreshold = 255;
	params.thresholdStep = 10;
	params.minArea = 10;
	//params.minConvexity = 0.3;  
	//params.minInertiaRatio = 0.01;  
	params.maxArea = 1000;
	//params.maxConvexity = 10;  
	params.filterByColor = /*false*/true;
	params.blobColor = 255; //Set blobColor = 0 to select darker blobs, and blobColor = 255 for lighter blobs.
							//params.filterByCircularity = false;  


	SimpleBlobDetector detector(params);
	detector.create("SimpleBlob");

	if (detector.empty()/* || descriptor_extractor.empty()*/)
	{
		throw runtime_error("fail to create detector!");
		return 0;
	}

	TraversFolderForCertainFiles(pInputDir, pExt, filenames);
	if (filenames.size() == 0)
		return 0;

	CreateMultipleDirectory(pOutputDir);

	for (int i = 0; i < filenames.size(); i++)
	{
		string iFile = pInputDir;
		iFile += "/";
		iFile += filenames[i];

		Mat img1 = imread(iFile/*, CV_LOAD_IMAGE_COLOR*/);//宏定义时CV_LOAD_IMAGE_GRAYSCALE=0，也就是读取灰度图像

		vector<KeyPoint> keyPoints;
		detector.detect(img1, keyPoints);
		cout << "img:" << filenames[i] << " points" << keyPoints.size() << endl;
		//		cout << keyPoints.size() << endl;

		Mat img_keypoints;
		drawKeypoints(img1, keyPoints, img_keypoints, Scalar::all(-1), 0);

		char result_name[128], suffix_name[8];
		GetFileName(filenames[i].c_str(), result_name, suffix_name);

		string out_name;
		char buf[10];
		sprintf(buf, "_keys");

		out_name += pOutputDir;
		out_name += "/";
		out_name += result_name;
		out_name += buf;
		out_name += suffix_name;

		imwrite(out_name, img_keypoints);


		// 		namedWindow("blobs"/*, CV_WINDOW_NORMAL*/);
		// 		imshow("blobs", img_keypoints);
		// 		waitKey(0);

		//波段分离
		/*		vector<Mat> channels;
		split(img1, channels);//分离色彩通道

		char result_name[128], suffix_name[8];
		GetFileName(filenames[i].c_str(), result_name, suffix_name);

		for (int j = 0; j < channels.size(); j++)
		{
		string chan_name;
		char buf[10];
		if(j==0)
		sprintf(buf, "_b");
		else if(j==1)
		sprintf(buf, "_g");
		else if(j==2)
		sprintf(buf, "_r");
		else
		sprintf(buf, "_b%d", j);

		chan_name += pOutputDir;
		chan_name += "/";
		chan_name += result_name;
		chan_name += buf;
		chan_name += suffix_name;

		imwrite(chan_name, channels[j]);


		}*/

	}
	//#endif

	//	waitKey(0);
	byebye(argc == 1);
	return 0;
}


