#include "Calibration.h"
#include "Header.h"



// �摜����`�F�b�J�[�p�^�[���̌�_���擾
bool Calibration::getCheckerCorners(std::vector<cv::Point2f> &imagePoint, const cv::Mat &image, cv::Mat &draw_image)
{
	// ��_���o
	bool detect = cv::findChessboardCorners( image, checkerPattern, imagePoint);

	// ���o�_�̕`��
	image.copyTo(draw_image);
	if(detect) {

		// �T�u�s�N�Z������
		cv::Mat	gray;
		cv::cvtColor( image, gray, cv::COLOR_BGR2GRAY );
		cv::cornerSubPix( gray, imagePoint, cv::Size( 11, 11 ), cv::Size( -1, -1 ), cv::TermCriteria( cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 20, 0.001 ) );

		cv::drawChessboardCorners( draw_image, checkerPattern, imagePoint, true );
	} else {
		cv::drawChessboardCorners( draw_image, checkerPattern, imagePoint, false );
	}

	return detect;
}


// �ē��e�덷�̌v�Z
void Calibration::calcReprojectionError(const std::vector<std::vector<cv::Point3f>> &worldPoints, const std::vector<std::vector<cv::Point2f>> &cameraPoints, const std::vector<std::vector<cv::Point2f>> &projectorPoints,
											double &cam_error, double &proj_error)
{
	cv::Mat camera_R = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat camera_T = cv::Mat::zeros(3, 1, CV_64F);

	// �J�����̍ē��e�덷
	for(int i=0; i<worldPoints.size(); ++i)
	{
		cv::Mat rvec, tvec;
		cv::solvePnP(worldPoints[i], cameraPoints[i], cam_K, cam_dist, rvec, tvec);		// �`�F�b�J�[�p�^�[���̈ʒu���o

		cv::Mat rmat;
		cv::Rodrigues(rvec, rmat);

		// �`�F�b�J�[�p�^�[�����S����J�������S�ɍ��W�ϊ�
		rmat = rmat.t();	// �]�u�s��

		cv::Mat extrinsic(4, 4, CV_64F);
		extrinsic.at<double>(0,0) = rmat.at<double>(0,0);
		extrinsic.at<double>(0,1) = rmat.at<double>(0,1);
		extrinsic.at<double>(0,2) = rmat.at<double>(0,2);
		extrinsic.at<double>(1,0) = rmat.at<double>(1,0);
		extrinsic.at<double>(1,1) = rmat.at<double>(1,1);
		extrinsic.at<double>(1,2) = rmat.at<double>(1,2);
		extrinsic.at<double>(2,0) = rmat.at<double>(2,0);
		extrinsic.at<double>(2,1) = rmat.at<double>(2,1);
		extrinsic.at<double>(2,2) = rmat.at<double>(2,2);
		extrinsic.at<double>(0,3) = cv::Mat(-rmat*tvec).at<double>(0,0);
		extrinsic.at<double>(1,3) = cv::Mat(-rmat*tvec).at<double>(1,0);
		extrinsic.at<double>(2,3) = cv::Mat(-rmat*tvec).at<double>(2,0);
		extrinsic.at<double>(3,0) = 0.0;
		extrinsic.at<double>(3,1) = 0.0;
		extrinsic.at<double>(3,2) = 0.0;
		extrinsic.at<double>(3,3) = 1.0;

		// �`�F�b�J�[�p�^�[���̌�_�ʒu
		std::vector<cv::Point3f> new_worldPoint;
		for(int j=0; j<worldPoints[0].size(); ++j)
		{
			cv::Mat checker_pos = extrinsic.inv() * cv::Mat((cv::Mat_<double>(4,1) << worldPoints[i][j].x, worldPoints[i][j].y, worldPoints[i][j].z, 1.0));		// �`�F�b�J�[�p�^�[���̈ʒu
			new_worldPoint.emplace_back(cv::Point3f(checker_pos.at<double>(0)/checker_pos.at<double>(3), checker_pos.at<double>(1)/checker_pos.at<double>(3), checker_pos.at<double>(2)/checker_pos.at<double>(3)));
		}

		// �J�������W�ւ̓��e
		std::vector<cv::Point2f> cam_projection;
		cv::projectPoints(new_worldPoint, camera_R, camera_T, cam_K, cam_dist, cam_projection);

		// �v���W�F�N�^���W�ւ̓��e
		std::vector<cv::Point2f> proj_projection;
		cv::projectPoints(new_worldPoint, R, T, proj_K, proj_dist, proj_projection);

		// �J�������W�ւ̍ē��e�덷
		for(int j=0; j<cameraPoints[0].size(); ++j)
		{
			cam_error += std::sqrt((cameraPoints[i][j].x - cam_projection[j].x)*(cameraPoints[i][j].x - cam_projection[j].x) + (cameraPoints[i][j].y - cam_projection[j].y)*(cameraPoints[i][j].y - cam_projection[j].y));
		}

		// �v���W�F�N�^���W�ւ̍ē��e�덷
		for(int j=0; j<projectorPoints[0].size(); ++j)
		{
			proj_error += std::sqrt((projectorPoints[i][j].x - proj_projection[j].x)*(projectorPoints[i][j].x - proj_projection[j].x) + (projectorPoints[i][j].y - proj_projection[j].y)*(projectorPoints[i][j].y - proj_projection[j].y));
		}
	}

	double sum = worldPoints.size() * worldPoints[0].size();

	cam_error /= sum;
	proj_error /= sum;
}


// �v���W�F�N�^�ƃJ�����̃L�����u���[�V����
void Calibration::proCamCalibration(const std::vector<std::vector<cv::Point3f>> &worldPoints, const std::vector<std::vector<cv::Point2f>> &cameraPoints, const std::vector<std::vector<cv::Point2f>> &projectorPoints,
										const cv::Size &camSize, const cv::Size &projSize)
{
	// �J�����L�����u���[�V����
	double cam_error = cv::calibrateCamera(worldPoints, cameraPoints, camSize, cam_K, cam_dist, cam_R, cam_T, cv::CALIB_FIX_K3, 
									cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, DBL_EPSILON));

	// �v���W�F�N�^�L�����u���[�V����
	double proj_error = cv::calibrateCamera(worldPoints, projectorPoints, projSize, proj_K, proj_dist, proj_R, proj_T, cv::CALIB_FIX_K3, 
									cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, DBL_EPSILON));

	// �X�e���I�œK��
	double stereo_error = cv::stereoCalibrate(worldPoints, cameraPoints, projectorPoints, cam_K, cam_dist, proj_K, proj_dist, camSize, R, T, E, F, 
                                                /*cv::CALIB_FIX_INTRINSIC*/  cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, DBL_EPSILON), cv::CALIB_USE_INTRINSIC_GUESS+cv::CALIB_FIX_K3);

	// �œK����̍ē��e�덷�̌v�Z
	double cam_error2 = 0;
	double proj_error2 = 0;
	calcReprojectionError(worldPoints, cameraPoints, projectorPoints, cam_error2, proj_error2);
	
	// ����
	std::cout << "***** Calibration results *****" << std::endl << std::endl;

	std::cout	<< "Camera Calibration results:" << std::endl
				<< " - Reprojection error: " << cam_error << std::endl
				<< " - Reprojection error2: " << cam_error2 << std::endl
				<< " - K:\n" << cam_K << std::endl
				<< " - Distortion:" << cam_dist << std::endl << std::endl;

	std::cout	<< "Projector Calibration results:" << std::endl
				<< " - Reprojection error: " << proj_error << std::endl
				<< " - Reprojection error2: " << proj_error2 << std::endl
				<< " - K:\n" << proj_K << std::endl
				<< " - Distortion:" << proj_dist << std::endl << std::endl;

	std::cout	<< "Stereo Calibration results:" << std::endl
				<< " - Reprojection error: " << stereo_error << std::endl
				<< " - R:\n" << R << std::endl
				<< " - T:" << T << std::endl << std::endl;


	// ���ʂ̕ۑ�
	cv::FileStorage fs("calibration.xml", cv::FileStorage::WRITE);
	fs << "cam_reprojection_error" << cam_error
	   << "cam_reprojection_error2" << cam_error2
	   << "proj_reprojection_error" << proj_error
	   << "proj_reprojection_error2" << proj_error2
	   << "stereo_reprojection_error" << stereo_error
	   << "cam_K" << cam_K << "cam_dist" << cam_dist
	   << "cam_R" << cam_R << "cam_T" << cam_T
       << "proj_K" << proj_K << "proj_dist" << proj_dist
       << "proj_R" << proj_R << "proj_T" << proj_T
	   << "R" << R << "T" << T << "E" << E << "F" << F;
	fs.release();

	calib_flag = true;
}


// �L�����u���[�V�������ʂ̓ǂݍ���
void Calibration::loadCalibParam(const std::string &fileName)
{
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);

	cvfs["cam_K"] >> cam_K;
	cvfs["cam_dist"] >> cam_dist;
	cvfs["cam_R"] >> cam_R;
	cvfs["cam_T"] >> cam_T;
	cvfs["proj_K"] >> proj_K;
	cvfs["proj_dist"] >> proj_dist;
	cvfs["proj_R"] >> proj_R;
	cvfs["proj_T"] >> proj_T;
	cvfs["R"] >> R;
	cvfs["T"] >> T;
	cvfs["E"] >> E;
	cvfs["F"] >> F;

	calib_flag = true;
}


// �������e�ϊ��s��̎擾(�J����)
cv::Mat Calibration::getCamPerspectiveMat()
{
	// ��]�ƕ��i������
	cv::Mat extrinsic = cv::Mat::eye(4, 4, CV_64F);

	// �����p�����[�^�̕ό`
	cv::Mat intrinsic(3, 4, CV_64F);
	intrinsic.at<double>(0,0) = cam_K.at<double>(0,0);
	intrinsic.at<double>(0,1) = cam_K.at<double>(0,1);
	intrinsic.at<double>(0,2) = cam_K.at<double>(0,2);
	intrinsic.at<double>(1,0) = cam_K.at<double>(1,0);
	intrinsic.at<double>(1,1) = cam_K.at<double>(1,1);
	intrinsic.at<double>(1,2) = cam_K.at<double>(1,2);
	intrinsic.at<double>(2,0) = cam_K.at<double>(2,0);
	intrinsic.at<double>(2,1) = cam_K.at<double>(2,1);
	intrinsic.at<double>(2,2) = cam_K.at<double>(2,2);
	intrinsic.at<double>(0,3) = 0.0;
	intrinsic.at<double>(1,3) = 0.0;
	intrinsic.at<double>(2,3) = 0.0;

	return intrinsic * extrinsic;
}


// �������e�ϊ��s��̎擾(�v���W�F�N�^)
cv::Mat Calibration::getProjPerspectiveMat()
{
	// ��]�ƕ��i������
	cv::Mat extrinsic(4, 4, CV_64F);
	extrinsic.at<double>(0,0) = R.at<double>(0,0);
	extrinsic.at<double>(0,1) = R.at<double>(0,1);
	extrinsic.at<double>(0,2) = R.at<double>(0,2);
	extrinsic.at<double>(1,0) = R.at<double>(1,0);
	extrinsic.at<double>(1,1) = R.at<double>(1,1);
	extrinsic.at<double>(1,2) = R.at<double>(1,2);
	extrinsic.at<double>(2,0) = R.at<double>(2,0);
	extrinsic.at<double>(2,1) = R.at<double>(2,1);
	extrinsic.at<double>(2,2) = R.at<double>(2,2);
	extrinsic.at<double>(0,3) = T.at<double>(0,0);
	extrinsic.at<double>(1,3) = T.at<double>(1,0);
	extrinsic.at<double>(2,3) = T.at<double>(2,0);
	extrinsic.at<double>(3,0) = 0.0;
	extrinsic.at<double>(3,1) = 0.0;
	extrinsic.at<double>(3,2) = 0.0;
	extrinsic.at<double>(3,3) = 1.0;

	// �����p�����[�^�̕ό`
	cv::Mat intrinsic(3, 4, CV_64F);
	intrinsic.at<double>(0,0) = proj_K.at<double>(0,0);
	intrinsic.at<double>(0,1) = proj_K.at<double>(0,1);
	intrinsic.at<double>(0,2) = proj_K.at<double>(0,2);
	intrinsic.at<double>(1,0) = proj_K.at<double>(1,0);
	intrinsic.at<double>(1,1) = proj_K.at<double>(1,1);
	intrinsic.at<double>(1,2) = proj_K.at<double>(1,2);
	intrinsic.at<double>(2,0) = proj_K.at<double>(2,0);
	intrinsic.at<double>(2,1) = proj_K.at<double>(2,1);
	intrinsic.at<double>(2,2) = proj_K.at<double>(2,2);
	intrinsic.at<double>(0,3) = 0.0;
	intrinsic.at<double>(1,3) = 0.0;
	intrinsic.at<double>(2,3) = 0.0;

	return intrinsic * extrinsic;
}


// �J�����ʒu�����[���h���W�Ƃ����ۂ̑Ώە��̂̈ʒu�̎擾
void Calibration::getCameraWorldPoint(std::vector<cv::Point3f> &camWorldPoint, const std::vector<cv::Point2f> &imagePoint)
{
	cv::Mat rvec, tvec, rmat;

	// �`�F�b�J�[�p�^�[���̈ʒu���o
	cv::solvePnP(worldPoint, imagePoint, cam_K, cv::Mat(), rvec, tvec);		

	cv::Rodrigues(rvec, rmat);		// ��]�s��ɕϊ�

	// �`�F�b�J�[�p�^�[�����S����J�������S�ɍ��W�ϊ�
	rmat = rmat.t();	// �]�u�s��

	cv::Mat extrinsic(4, 4, CV_64F);
	extrinsic.at<double>(0,0) = rmat.at<double>(0,0);
	extrinsic.at<double>(0,1) = rmat.at<double>(0,1);
	extrinsic.at<double>(0,2) = rmat.at<double>(0,2);
	extrinsic.at<double>(1,0) = rmat.at<double>(1,0);
	extrinsic.at<double>(1,1) = rmat.at<double>(1,1);
	extrinsic.at<double>(1,2) = rmat.at<double>(1,2);
	extrinsic.at<double>(2,0) = rmat.at<double>(2,0);
	extrinsic.at<double>(2,1) = rmat.at<double>(2,1);
	extrinsic.at<double>(2,2) = rmat.at<double>(2,2);
	extrinsic.at<double>(0,3) = cv::Mat(-rmat*tvec).at<double>(0,0);
	extrinsic.at<double>(1,3) = cv::Mat(-rmat*tvec).at<double>(1,0);
	extrinsic.at<double>(2,3) = cv::Mat(-rmat*tvec).at<double>(2,0);
	extrinsic.at<double>(3,0) = 0.0;
	extrinsic.at<double>(3,1) = 0.0;
	extrinsic.at<double>(3,2) = 0.0;
	extrinsic.at<double>(3,3) = 1.0;

	// �`�F�b�J�[�p�^�[���̌�_�ʒu
	for(int i=0; i<worldPoint.size(); ++i)
	{
		cv::Mat checker_pos = extrinsic.inv() * cv::Mat((cv::Mat_<double>(4,1) << worldPoint[i].x, worldPoint[i].y, worldPoint[i].z, 1.0));		// �`�F�b�J�[�p�^�[���̈ʒu
		camWorldPoint.emplace_back(cv::Point3f(checker_pos.at<double>(0)/checker_pos.at<double>(3), checker_pos.at<double>(1)/checker_pos.at<double>(3), checker_pos.at<double>(2)/checker_pos.at<double>(3)));
	}
}


// 3��������
void Calibration::reconstruction(std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, const std::vector<int> &flag)
{
	// �������e�ϊ��s��
	cv::Mat cam_pers = getCamPerspectiveMat();
	cv::Mat proj_pers = getProjPerspectiveMat();


	static cv::Mat f(4, 1, CV_64FC1, cv::Scalar(0.0));
	static cv::Mat q(4, 3, CV_64FC1, cv::Scalar(0.0));
	static cv::Mat v(3, 1, CV_64FC1, cv::Scalar(0.0));

	// object space�̍ŏ����ɂ��3��������
	for(int i=0; i<projPoint.size(); ++i)
	{
		if(flag[i] == 1)
		{
			f.at<double>(0, 0) = imagePoint[i].x * cam_pers.at<double>(2, 3) - cam_pers.at<double>(0, 3);
			f.at<double>(1, 0) = imagePoint[i].y * cam_pers.at<double>(2, 3) - cam_pers.at<double>(1, 3);
			f.at<double>(2, 0) = projPoint[i].x * proj_pers.at<double>(2, 3) - proj_pers.at<double>(0, 3);
			f.at<double>(3, 0) = projPoint[i].y * proj_pers.at<double>(2, 3) - proj_pers.at<double>(1, 3);

			q.at<double>(0, 0) = cam_pers.at<double>(0, 0) - cam_pers.at<double>(2, 0) * imagePoint[i].x;
			q.at<double>(0, 1) = cam_pers.at<double>(0, 1) - cam_pers.at<double>(2, 1) * imagePoint[i].x;
			q.at<double>(0, 2) = cam_pers.at<double>(0, 2) - cam_pers.at<double>(2, 2) * imagePoint[i].x;
			q.at<double>(1, 0) = cam_pers.at<double>(1, 0) - cam_pers.at<double>(2, 0) * imagePoint[i].y;
			q.at<double>(1, 1) = cam_pers.at<double>(1, 1) - cam_pers.at<double>(2, 1) * imagePoint[i].y;
			q.at<double>(1, 2) = cam_pers.at<double>(1, 2) - cam_pers.at<double>(2, 2) * imagePoint[i].y;
			q.at<double>(2, 0) = proj_pers.at<double>(0, 0) - proj_pers.at<double>(2, 0) * projPoint[i].x;
			q.at<double>(2, 1) = proj_pers.at<double>(0, 1) - proj_pers.at<double>(2, 1) * projPoint[i].x;
			q.at<double>(2, 2) = proj_pers.at<double>(0, 2) - proj_pers.at<double>(2, 2) * projPoint[i].x;
			q.at<double>(3, 0) = proj_pers.at<double>(1, 0) - proj_pers.at<double>(2, 0) * projPoint[i].y;
			q.at<double>(3, 1) = proj_pers.at<double>(1, 1) - proj_pers.at<double>(2, 1) * projPoint[i].y;
			q.at<double>(3, 2) = proj_pers.at<double>(1, 2) - proj_pers.at<double>(2, 2) * projPoint[i].y;

			v = q.inv(cv::DECOMP_SVD) * f;
			reconstructPoint.emplace_back(cv::Point3f(v.at<double>(0, 0), v.at<double>(1, 0), v.at<double>(2, 0)));
		}
		else
		{
			reconstructPoint.emplace_back(cv::Point3f(-1, -1, -1));
		}
	}
}


// 3�����_�Q�̕`��
void Calibration::pointCloudRender(const std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &imagePoint, const cv::Mat &image, 
								std::string &windowName, const cv::Mat& R, const cv::Mat& t)
{

	//cv::Mat viewer(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3, cv::Scalar(0));
	cv::Mat viewer(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0));

	// 2�������ʂ֓��e
	std::vector<cv::Point2f> pt;
    cv::projectPoints(reconstructPoint, R, t, proj_K, cv::Mat(), pt); 
	
	// �^��Z�o�b�t�@
	cv::Mat z_buffer(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar(0.0));

	int count = 0;
	bool cut_flag = false;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());

	// �F�t��
	for(int i=0; i<pt.size(); ++i)
	{
		count = 0;
		cut_flag = false;

		int pt_x = (int)(pt[i].x+0.5);
		int pt_y = (int)(pt[i].y+0.5);
		if(pt_x >= 0 && pt_x < viewer.cols && pt_y >= 0 && pt_y < viewer.rows)
		{
			if(z_buffer.at<double>(pt_y,pt_x) == 0.0 || z_buffer.at<double>(pt_y,pt_x) > reconstructPoint[i].z )
			{
				int image_x = (int)(imagePoint[i].x+0.5);
				int image_y = (int)(imagePoint[i].y+0.5);
				//�J������f��̐F��t����
				//int image_x = i % CAMERA_WIDTH;
				//int image_y = (int)(i / CAMERA_WIDTH);

				viewer.at<uchar>(pt_y, 3*pt_x+0) = image.at<uchar>(image_y, 3*image_x+0);
				viewer.at<uchar>(pt_y, 3*pt_x+1) = image.at<uchar>(image_y, 3*image_x+1);
				viewer.at<uchar>(pt_y, 3*pt_x+2) = image.at<uchar>(image_y, 3*image_x+2);
				z_buffer.at<double>(pt_y,pt_x) = reconstructPoint[i].z;
			}
		}
	}

	cv::namedWindow(windowName, CV_WINDOW_FREERATIO);
	cv::imshow(windowName, viewer);
}