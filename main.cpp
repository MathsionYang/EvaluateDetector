#pragma region ������������������
//#include <opencv2/features2d.hpp>
//#include <opencv2/opencv.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/xfeatures2d.hpp>
//#include <iostream>
//
//using namespace std;
//using namespace cv;
//
//void ReadFundamenta(char* Hpath, Mat &H);
//void checkmacth(vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2, vector<cv::DMatch>  matches_all, Mat TrueTranH);
//Point2f GetpointTtoR(const Point2f TPoint, const Mat H);
//
//int main(void)
//{
//	// Read img1 and img2 from path 
//	string filename1 = "img\\boat\\boat1.png";
//	string filename2 = "img\\boat\\boat2.png";
//	Mat img1 = imread(filename1);
//	Mat img2 = imread(filename2);
//
//	// keypoints1 keypoints2 is initializing 
//	vector<cv::KeyPoint> *keypoints1 = NULL, *keypoints2 = NULL;
//
//	// Homography MatRix
//	char* Hpath = "img\\boat\\H1to2p.txt";
//	Mat H1to2 = Mat::zeros(3, 3, CV_64F);
//	ReadFundamenta(Hpath, H1to2);
//
//	float repeatability = 0.0;
//	int correspCount = 0;
//
//	Ptr<Feature2D> _fdetector_sift = xfeatures2d::SIFT::create();
//	Ptr<Feature2D> _fdetector_surf = xfeatures2d::SURF::create();
//	Ptr<Feature2D> _fdetector_orb = ORB::create();
//	vector<cv::KeyPoint> keypt1, keypt2;
//	Mat _desc1, _desc2;
//
//	_fdetector_sift->detectAndCompute(img1, Mat(), keypt1, _desc1);
//	_fdetector_sift->detectAndCompute(img2, Mat(), keypt2, _desc2);
//	BFMatcher matcher(NORM_HAMMING);
//	//BFMatcher matcher;
//	vector<DMatch>  matches_1_2, matches_2_1;
//	vector<cv::DMatch>  matches_all;
//	matcher.match(_desc1, _desc2, matches_1_2);
//	matcher.match(_desc2, _desc1, matches_2_1);
//	for (int i = 0; i < matches_1_2.size(); i++)
//	{
//		if (i == matches_2_1[matches_1_2[i].trainIdx].trainIdx)
//		{
//			matches_all.push_back(matches_1_2[i]);
//		}
//	}
//	cout << "��ʼƥ����Ŀ is" << matches_all.size() << endl;
//	
//	checkmacth(keypt1, keypt2, matches_all, H1to2);
//	//////////************************************��������************************************//////////
//	/*cout << "keypt1 is" << keypt1.size() << " keypt2 is :" << keypt2.size() << endl;
//	evaluateFeatureDetector(img1, img2, H1to2, keypoints1, keypoints2, repeatability, correspCount, _fdetector_sift);
//	cout << "repeatability is" << repeatability << "correspCount is :" << correspCount << endl;*/
//	//////////************************************��������************************************//////////
//	getchar();
//	return 0;
//}
//void ReadFundamenta(char* Hpath, Mat &H)
//{
//	FILE *fp1 = fopen(Hpath, "r");
//	for (int i = 0; i < 3; i++) {
//		float a1, a2, a3;
//		fscanf(fp1, "%f %f %f \n", &a1, &a2, &a3);
//		H.at<double>(i, 0) = a1;
//		H.at<double>(i, 1) = a2;
//		H.at<double>(i, 2) = a3;
//	}
//	std::fclose(fp1);
//}
//void checkmacth(vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2, vector<cv::DMatch>  matches_all,Mat TrueTranH)
//{
//	int count = matches_all.size();
//	double sumdis = 0;
//	for (int i = 0; i < count; i++) {
//		Point2f Tpoint = kp1[matches_all[i].queryIdx].pt;
//		Point2f Rpoint = kp2[matches_all[i].trainIdx].pt;
//		Point2f TRpoint = GetpointTtoR(Tpoint, TrueTranH);
//		double dispoint = sqrtf((TRpoint.x - Rpoint.x)*(TRpoint.x - Rpoint.x) + (TRpoint.y - Rpoint.y)*(TRpoint.y - Rpoint.y));
//		if (dispoint <= 0.8)
//		{
//			sumdis = sumdis + 1;
//		}
//	}
//	cout << "С�ڰ�����صĸ���:" << sumdis << endl;
//
//}
//Point2f GetpointTtoR(const Point2f TPoint, const Mat H)
//{
//	Mat MT, MTR;
//	MT.create(3, 1, CV_64F);
//	MTR.create(3, 1, CV_64F);
//	MT.at<double>(0, 0) = TPoint.x;
//	MT.at<double>(1, 0) = TPoint.y;
//	MT.at<double>(2, 0) = 1;
//	Mat tmp = H * MT;
//
//	MTR.at<double>(0, 0) = tmp.at<double>(0, 0) / tmp.at<double>(2, 0);
//	MTR.at<double>(1, 0) = tmp.at<double>(1, 0) / tmp.at<double>(2, 0);
//	Point2f TR = Point2f(MTR.at<double>(0, 0), MTR.at<double>(1, 0));
//	return TR;
//}

#pragma endregion



#pragma region SURF��SIFT��ORB���ܶԱ�

//#include <opencv2/core/utility.hpp>
//#include "opencv2/imgproc.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include <iostream>
//#include <ctype.h>
//
//#define DATESET_COUNT 4
//#define METHOD_COUNT 3
//using namespace cv;
//using namespace std;
//using namespace xfeatures2d;
//Point2f GetpointTtoR(const Point2f TPoint, const Mat H)
//{
//	Mat MT, MTR;
//	MT.create(3, 1, CV_64F);
//	MTR.create(3, 1, CV_64F);
//	MT.at<double>(0, 0) = TPoint.x;
//	MT.at<double>(1, 0) = TPoint.y;
//	MT.at<double>(2, 0) = 1;
//	Mat tmp = H * MT;
//
//	MTR.at<double>(0, 0) = tmp.at<double>(0, 0) / tmp.at<double>(2, 0);
//	MTR.at<double>(1, 0) = tmp.at<double>(1, 0) / tmp.at<double>(2, 0);
//	Point2f TR = Point2f(MTR.at<double>(0, 0), MTR.at<double>(1, 0));
//	return TR;
//}
//
//
//void checkmacth(vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2, vector<cv::DMatch>  matches_all, Mat TrueTranH)
//{
//	int count = matches_all.size();
//	double sumdis = 0;
//	for (int i = 0; i < count; i++) {
//		Point2f Tpoint = kp1[matches_all[i].queryIdx].pt;
//		Point2f Rpoint = kp2[matches_all[i].trainIdx].pt;
//		Point2f TRpoint = GetpointTtoR(Tpoint, TrueTranH);
//		double dispoint = sqrtf((TRpoint.x - Rpoint.x)*(TRpoint.x - Rpoint.x) + (TRpoint.y - Rpoint.y)*(TRpoint.y - Rpoint.y));
//		if (dispoint <= 0.8)
//		{
//			sumdis = sumdis + 1;
//		}
//	}
//	cout << "��ȷ��:" << sumdis / count << endl;
//
//}
//void main()
//{
//	string strDateset[DATESET_COUNT];
//	strDateset[0] = "boat"; strDateset[1] = "car"; strDateset[2] = "graf"; strDateset[3] = "ubc";
//	string strMethod[METHOD_COUNT];
//	strMethod[0] = "SIFT"; strMethod[1] = "SURF"; strMethod[2] = "ORB";
//	////�ݹ��ȡĿ¼��ȫ���ļ�
//	Mat descriptors1;
//	std::vector<KeyPoint> keypoints1;
//	Mat descriptors2;
//	std::vector<KeyPoint> keypoints2;
//	std::vector< DMatch > matches;
//	std::vector< DMatch > good_matches;
//	////����ģ������
//	int innersize = 0;
//	Mat img1;
//	Mat imgn;
//	int64 t = getTickCount();
//	std::cout << "�ӣɣƣԡ��ӣգңơ��ϣң¡��㷨����ʵ�鿪ʼ" << endl;
//	//��������������Ѱ�ҷ���
//	for (int imethod = 0; imethod < METHOD_COUNT; imethod++)
//	{
//
//
//		string _strMethod = strMethod[imethod];
//		std::cout << "��ʼ����" << _strMethod << "����" << endl;
//		//��������·��
//		for (int idateset = 0; idateset < DATESET_COUNT; idateset++)
//		{
//			//��ò���ͼƬ���Ե�ַ
//			string path = "img/" + strDateset[idateset];
//			std::cout << "���ݼ�Ϊ" << strDateset[idateset] << endl;
//			//��õ������ݼ��е�ͼƬ
//
//				//ʹ��img1�Ա����µ�ͼƬ���ó����    
//			img1 = imread((path + "/1.jpg"), 0);
//			imgn = imread((path + "/2.jpg"), 0);
//			//�����������㷨����ƥ�䷽��
//			Ptr<Feature2D>  extractor;
//			BFMatcher matcher;
//			switch (imethod)
//			{
//			case 0: //"SIFT"
//				extractor = SIFT::create(5000);
//				matcher = BFMatcher(NORM_L2);
//				break;
//			case 1: //"SURF"
//				extractor = SURF::create(5000);
//				matcher = BFMatcher(NORM_L2);
//				break;
//
//			case 2: //"ORB"
//				extractor = ORB::create(5000);
//				matcher = BFMatcher(NORM_HAMMING);
//				break;
//
//			}
//			try
//			{
//				extractor->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
//				extractor->detectAndCompute(imgn, Mat(), keypoints2, descriptors2);
//				matcher.match(descriptors1, descriptors2, matches);
//			}
//			catch (Exception* e)
//			{
//				cout << " ��������ȡʱ�������� " << endl;
//				continue;
//			}
//
//			//����������д�ƥ��
//			double max_dist = 0;
//			double min_dist = 100;
//			for (int a = 0; a < matches.size(); a++)
//			{
//				double dist = matches[a].distance;
//				if (dist < min_dist) min_dist = dist;
//				if (dist > max_dist) max_dist = dist;
//			}
//			for (int a = 0; a < matches.size(); a++)
//			{
//				if (matches[a].distance <= max(2 * min_dist, 0.02))
//					good_matches.push_back(matches[a]);
//			}
//			if (good_matches.size() < 4)
//			{
//				cout << " ��Ч��������ĿС��4������ƥ��ʧ�� " << endl;
//				continue;
//			}
//			//ͨ��RANSAC�����������е�������Խ��С��ᴿ��
//			std::vector<Point2f> obj;
//			std::vector<Point2f> scene;
//			for (int a = 0; a < (int)good_matches.size(); a++)
//			{
//				//�ֱ�������good_matches��Ӧ�ĵ��ѹ������,ֻ��Ҫѹ������Ϣ�Ϳ���
//				obj.push_back(keypoints1[good_matches[a].queryIdx].pt);
//				scene.push_back(keypoints2[good_matches[a].trainIdx].pt);
//			}
//			//���㵥Ӧ������calib3d��)
//			Mat H;
//			try
//			{
//				H = findHomography(obj, scene, CV_RANSAC);
//			}
//			catch (Exception* e)
//			{
//				cout << " findHomographyʧ�� " << endl;
//				continue;
//			}
//			if (H.rows < 3)
//			{
//				cout << " findHomographyʧ�� " << endl;
//				continue;
//			}
//			//�����ڵ���Ŀ
//			Mat matObj;
//			Mat matScene;
//			CvMat* pcvMat = &(CvMat)H;
//			const double* Hmodel = pcvMat->data.db;
//			double Htmp = Hmodel[6];
//			for (int isize = 0; isize < obj.size(); isize++)
//			{
//				double ww = 1. / (Hmodel[6] * obj[isize].x + Hmodel[7] * obj[isize].y + 1.);
//				double dx = (Hmodel[0] * obj[isize].x + Hmodel[1] * obj[isize].y + Hmodel[2])*ww - scene[isize].x;
//				double dy = (Hmodel[3] * obj[isize].x + Hmodel[4] * obj[isize].y + Hmodel[5])*ww - scene[isize].y;
//				float err = (float)(dx*dx + dy * dy); //3������֮����Ϊ��ͬһ����
//				if (err < 9)
//				{
//					innersize = innersize + 1;
//				}
//			}
//			//ƥ����ȷ��
//			checkmacth(keypoints1, keypoints2, good_matches, H);
//
//			//��ӡ�ڵ�ռȫ��������ı���
//			float ff = (float)innersize / (float)good_matches.size();
//			cout << "�ڵ�ռȫ��������ı���:" << ff << endl;
//			//��ӡʱ��
//			cout << "ʱ��:" << ((getTickCount() - t) / getTickFrequency()) << endl;
//			t = getTickCount();
//			cout << "-----------------------------" << endl;
//			cout << "-----------------------------" << endl;
//			ff = 0;
//			innersize = 0;
//			matches.clear();
//			good_matches.clear();
//
//
//		}
//	}
//	getchar();
//	waitKey();
//	return;
//
//}
#pragma endregion


#pragma region LSD��Canny�Ա�ʵ��

#pragma endregion
