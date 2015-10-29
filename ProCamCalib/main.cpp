
#include "Graycode.h"
#include "Header.h"
#include "PGROpenCV.h"
#include "Calibration.h"


#define MASK_ADDRESS "./GrayCodeImage/mask.bmp"
#define IMAGE_DIRECTORY "./UseImage"
#define SAVE_DIRECTORY "./UseImage/resize"


TPGROpenCV	pgrOpenCV;

void eular2rot(double yaw,double pitch, double roll, cv::Mat& dest);

int main()
{
	GRAYCODE gc;

	printf("0�FProCam�Ԃ̊􉽑Ή����擾\n");
	printf("1�F���܂Ŏ擾�����􉽑Ή�����ProCam�L�����u���[�V����\n");
	printf("2�F�ۑ����Ă���􉽑Ή�����ProCam�L�����u���[�V����\n");
	printf("3�F�L�����u���[�V�����̓ǂݍ���\n");
	printf("4�F�L�����u���[�V�������ʂ��g���ē��e\n");
	printf("5�F�ē��e�덷�̌v��\n");
	printf("6�F3��������\n");

	printf("w�F�ҋ@���ɔ��摜�𓊉e���邩���Ȃ���\n");
	printf("\n");

	pgrOpenCV.init( FlyCapture2::PIXEL_FORMAT_BGR );
	//pgrOpenCV.setCameraParams(4.0);

	// �J�����摜�m�F�p
	char windowNameCamera[] = "camera";
	cv::namedWindow(windowNameCamera, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(windowNameCamera, 500, 300);

	static bool prjWhite = true;


	// �L�����u���[�V�����p
	Calibration calib(10, 7, 24.0);
	std::vector<std::vector<cv::Point3f>>	worldPoints;
	std::vector<std::vector<cv::Point2f>>	cameraPoints;
	std::vector<std::vector<cv::Point2f>>	projectorPoints;
	int calib_count = 0;


	// �L�[���͎�t�p�̖������[�v
	while(true){
		printf("====================\n");
		printf("��������͂��Ă�������....\n");
		int command;

		// �����摜��S��ʂœ��e�i�B�e�����m�F���₷�����邽�߁j
		pgrOpenCV.start();
		cv::Mat cam, cam2;
		while(true){
			// true�Ŕ��𓊉e�Afalse�Œʏ�̃f�B�X�v���C��\��
			if(prjWhite){
				cv::Mat white = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::namedWindow("white_black", 0);
				Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
				cv::imshow("white_black", white);
			}

			// �����̃L�[�����͂��ꂽ�烋�[�v�𔲂���
			command = cv::waitKey(33);
			if ( command > 0 ) break;

			pgrOpenCV.queryFrame();
			cam = pgrOpenCV.getVideo();
			cam.copyTo(cam2);

			//���₷���悤�ɓK���Ƀ��T�C�Y
			cv::resize(cam, cam, cv::Size(), 0.45, 0.45);
			cv::imshow(windowNameCamera, cam);
		}

		// �J�������~�߂�
		pgrOpenCV.stop();
		cv::destroyWindow("white_black");

		// ��������
		switch (command){

		case '0':
			{

				// �`�F�b�J�[�p�^�[���̌�_��`��(�J����)
				std::vector<cv::Point2f> imagePoint;
				std::vector<cv::Point2f> projPoint;
				cv::Mat draw_corner;
				bool detect_flag = calib.getCheckerCorners(imagePoint, cam2, draw_corner);
				cv::imshow( "Image Corner", draw_corner);

				// �J������Ń`�F�b�J�[�p�^�[�������o�ł�����
				if (detect_flag)
				{
					// �O���C�R�[�h���e
					gc.code_projection();
					gc.make_thresh();
					gc.makeCorrespondence();


					// �`�F�b�J�[�p�^�[���̌�_��`��(�v���W�F�N�^)
					cv::Mat dst;
					gc.getCorrespondSubPixelProjPoints(projPoint, imagePoint, 20);
					gc.transport_camera_projector(cam2,dst);

					if(imagePoint.size() == projPoint.size()) {
						cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, true );
					} else {
						cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, false );
					}

					cv::imshow("prj",dst);
					cv::imwrite("./prj.jpg", dst );


					// �`�F�b�J�[�p�^�[���̌�_�𓊉e
					cv::destroyWindow("white_black");
					cv::namedWindow("Corner", 0);
					Projection::MySetFullScrean(DISPLAY_NUMBER, "Corner");
					cv::imshow("Corner", dst);

					// �v���W�F�N�^��Ń`�F�b�J�[�p�^�[�������o�ł�����
					if(imagePoint.size() == projPoint.size())
					{
						std::cout << calib_count+1 << "��ڂ̌��o����" << std::endl;

						// �ǉ�
						worldPoints.emplace_back(calib.worldPoint);
						cameraPoints.emplace_back(imagePoint);
						projectorPoints.emplace_back(projPoint);

						// ���o�_�̕ۑ�
						std::string fileName = "checkerPoint" + std::to_string(calib_count) + ".xml";
						cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
						cv::write(fs,"world", calib.worldPoint);
						cv::write(fs,"camera", imagePoint);
						cv::write(fs,"projector", projPoint);

						calib_count++;
					}else{
						std::cout << "�v���W�F�N�^��̃`�F�b�J�[�p�^�[���̌��o���s" << std::endl;
					}
				} else {
					std::cout << "�J������̃`�F�b�J�[�p�^�[���̌��o���s" << std::endl;
				}

				cv::waitKey(0);
			}
			break;

		case '1':

			if(calib_count > 2)
			{
				std::cout << "�L�����u���[�V�������c" << std::endl;

				// �L�����u���[�V����
				calib.proCamCalibration(worldPoints, cameraPoints, projectorPoints, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), cv::Size(PROJECTOR_WIDTH, PROJECTOR_HEIGHT));

			} else {
				std::cout << "" << std::endl;
			}
			break;

		case '2':
			{
				worldPoints.clear();
				cameraPoints.clear();
				projectorPoints.clear();

				// �t�@�C���̒T��
				WIN32_FIND_DATA ffd;
				HANDLE hF;

				std::string imageFlieName = "checkerPoint*.*";
				hF = FindFirstFile( imageFlieName.c_str(), &ffd);
				if (hF != INVALID_HANDLE_VALUE) {
					// �t�H���_���̃t�@�C���̒T��
					do {
						std::string fullpath = ffd.cFileName;

						std::vector<cv::Point3f> worldPoint;
						std::vector<cv::Point2f> imagePoint;
						std::vector<cv::Point2f> projPoint;

						// xml�t�@�C���̓ǂݍ���
						cv::FileStorage cvfs(fullpath, cv::FileStorage::READ);

						cvfs["world"] >> worldPoint;
						cvfs["camera"] >> imagePoint;
						cvfs["projector"] >> projPoint;
						
						// �ǉ�
						worldPoints.emplace_back(worldPoint);
						cameraPoints.emplace_back(imagePoint);
						projectorPoints.emplace_back(projPoint);

					} while (FindNextFile(hF, &ffd ) != 0);
					FindClose(hF);
				}

				if(worldPoints.size() > 0)
				{
					std::cout << "�L�����u���[�V�������c" << std::endl;

					// �L�����u���[�V����
					calib.proCamCalibration(worldPoints, cameraPoints, projectorPoints, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), cv::Size(PROJECTOR_WIDTH, PROJECTOR_HEIGHT));

				}
			}
			break;

		case '3':

			std::cout << "�L�����u���[�V�������ʂ̓ǂݍ��ݒ��c" << std::endl;
			calib.loadCalibParam("calibration.xml");
				
			break;

		case '4':

			if(calib.calib_flag)
			{
				std::cout << "�`�F�b�J�[�p�^�[���ւ̍ē��e���c" << std::endl;
				
				// �������e�s��̐���
				cv::Mat cam_perspective = calib.getCamPerspectiveMat();
				cv::Mat proj_perspective = calib.getProjPerspectiveMat();

				pgrOpenCV.start();
				cv::Mat white = cv::Mat(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::Mat proj_img = cv::Mat(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
				while(true){
					// ���e�摜�̍X�V
					white.copyTo(proj_img);


					// �����̃L�[�����͂��ꂽ�烋�[�v�𔲂���
					command = cv::waitKey(33);
					if ( command > 0 ) break;

					pgrOpenCV.queryFrame();
					cam = pgrOpenCV.getVideo();
					
					// �`�F�b�J�[�p�^�[���̌�_��`��(�J����)
					cv::Mat cam3;
					cv::undistort(cam, cam3, calib.cam_K, calib.cam_dist);		// �c�ݏ���

					std::vector<cv::Point3f> worldPoint;
					std::vector<cv::Point2f> imagePoint;
					std::vector<cv::Point2f> projPoint;
					bool detect_flag = calib.getCheckerCorners(imagePoint, cam3, cam3);						// �`�F�b�J�[�p�^�[�����o

					if(detect_flag) {
						// �J�����𒆐S�Ƃ����`�F�b�J�[�p�^�[���̈ʒu�擾
						calib.getCameraWorldPoint(worldPoint, imagePoint);

						// �J�������W�ւ̓��e(�c�ݏ�����̂��ߘc�݃p�����[�^�͗p���Ȃ�)
						std::vector<cv::Point2f> cam_projection;
						cv::Mat cam_R = cv::Mat::eye(3, 3, CV_64F);
						cv::Mat cam_T = cv::Mat::zeros(3, 1, CV_64F);
						cv::projectPoints(worldPoint, cam_R, cam_T, calib.cam_K, cv::Mat(), cam_projection);

						// �J�����̍ē��e�덷
						float cam_error = 0;
						for(int i=0; i<imagePoint.size(); ++i)
						{
							cam_error += std::sqrt((imagePoint[i].x - cam_projection[i].x)*(imagePoint[i].x - cam_projection[i].x) + (imagePoint[i].y - cam_projection[i].y)*(imagePoint[i].y - cam_projection[i].y));
						}

						std::cout << "�J�����̍ē��e�덷�F" << cam_error/imagePoint.size() << std::endl;

						// �v���W�F�N�^���W�ւ̓��e(�c�݂���)
						cv::projectPoints(worldPoint, calib.R, calib.T, calib.proj_K, calib.proj_dist, projPoint);

						// �v���W�F�N�^���e�摜
						cv::drawChessboardCorners( proj_img, calib.checkerPattern, projPoint, true );
					}

					// �f���̓��e
					cv::namedWindow("white_black", 0);
					Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
					cv::imshow("white_black", proj_img);


					//���₷���悤�ɓK���Ƀ��T�C�Y
					cv::resize(cam3, cam3, cv::Size(), 0.45, 0.45);
					cv::imshow(windowNameCamera, cam3);
				}

				// �J�������~�߂�
				pgrOpenCV.stop();
				cv::destroyWindow("white_black");


			} else {
				std::cout << "�L�����u���[�V�����f�[�^������܂���" << std::endl;
			}
				
			break;

		case '5':
			{
				if(calib.calib_flag)
				{
					// �`�F�b�J�[�p�^�[���̌�_��`��(�J����)
					std::vector<cv::Point3f> worldPoint;
					std::vector<cv::Point2f> imagePoint;
					std::vector<cv::Point2f> projPoint;
					cv::Mat draw_corner;
					bool detect_flag = calib.getCheckerCorners(imagePoint, cam2, draw_corner);

					// �J�����̘c�ݏ���
					cv::Mat draw_corner2;
					cv::undistort(draw_corner, draw_corner2, calib.cam_K, calib.cam_dist);		// �c�ݏ���

					cv::imshow( "Image Corner", draw_corner2);

					// �J������Ń`�F�b�J�[�p�^�[�������o�ł�����
					if (detect_flag)
					{
						std::cout << "�ē��e�덷�̌v�����c" << std::endl;

						// �O���C�R�[�h���e
						gc.code_projection();
						gc.make_thresh();
						gc.makeCorrespondence();

						// �`�F�b�J�[�p�^�[���̌�_��`��(�v���W�F�N�^)
						cv::Mat dst;
						gc.getCorrespondSubPixelProjPoints(projPoint, imagePoint, 30);		// �Ή��_�̎擾
						gc.transport_camera_projector(cam2,dst);

						if(imagePoint.size() == projPoint.size()) {
							cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, true );
						} else {
							cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, false );
						}
						cv::imshow("prj",dst);

						
						// �Ή��_�̘c�ݏ���
						std::vector<cv::Point2f> undistort_imagePoint;
						std::vector<cv::Point2f> undistort_projPoint;
						cv::undistortPoints(imagePoint, undistort_imagePoint, calib.cam_K, calib.cam_dist);
						cv::undistortPoints(projPoint, undistort_projPoint, calib.proj_K, calib.proj_dist);
						for(int i=0; i<imagePoint.size(); ++i)
						{
							undistort_imagePoint[i].x = undistort_imagePoint[i].x * calib.cam_K.at<double>(0,0) + calib.cam_K.at<double>(0,2);
							undistort_imagePoint[i].y = undistort_imagePoint[i].y * calib.cam_K.at<double>(1,1) + calib.cam_K.at<double>(1,2);
							undistort_projPoint[i].x = undistort_projPoint[i].x * calib.proj_K.at<double>(0,0) + calib.proj_K.at<double>(0,2);
							undistort_projPoint[i].y = undistort_projPoint[i].y * calib.proj_K.at<double>(1,1) + calib.proj_K.at<double>(1,2);
						}

						// �J�����𒆐S�Ƃ����`�F�b�J�[�p�^�[���̈ʒu�擾
						calib.getCameraWorldPoint(worldPoint, undistort_imagePoint);

						// �J�������W�ւ̓��e(�c�ݏ�����̂��ߘc�݃p�����[�^�͗p���Ȃ�)
						std::vector<cv::Point2f> cam_projection;
						cv::Mat cam_R = cv::Mat::eye(3, 3, CV_64F);
						cv::Mat cam_T = cv::Mat::zeros(3, 1, CV_64F);
						cv::projectPoints(worldPoint, cam_R, cam_T, calib.cam_K, cv::Mat(), cam_projection);

						// �J�����̍ē��e�덷
						float cam_error = 0;
						for(int i=0; i<undistort_imagePoint.size(); ++i)
						{
							cam_error += std::sqrt((undistort_imagePoint[i].x - cam_projection[i].x)*(undistort_imagePoint[i].x - cam_projection[i].x) + (undistort_imagePoint[i].y - cam_projection[i].y)*(undistort_imagePoint[i].y - cam_projection[i].y));
						}

						std::cout << "�J�����̍ē��e�덷�F" << cam_error/undistort_imagePoint.size() << std::endl;


						// �v���W�F�N�^���W�ւ̓��e(�c�ݏ�����̂��ߘc�݃p�����[�^�͗p���Ȃ�)
						std::vector<cv::Point2f> proj_projection;
						cv::projectPoints(worldPoint, calib.R, calib.T, calib.proj_K, cv::Mat(), proj_projection);

						// �v���W�F�N�^�̍ē��e�덷
						float proj_error = 0;
						for(int i=0; i<undistort_projPoint.size(); ++i)
						{
							proj_error += std::sqrt((undistort_projPoint[i].x - proj_projection[i].x)*(undistort_projPoint[i].x - proj_projection[i].x) + (undistort_projPoint[i].y - proj_projection[i].y)*(undistort_projPoint[i].y - proj_projection[i].y));
						}

						std::cout << "�v���W�F�N�^�̍ē��e�덷�F" << proj_error/undistort_projPoint.size() << std::endl;


						// �v���W�F�N�^���e�摜
						cv::Mat proj_img = cv::Mat(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
						std::vector<cv::Point2f> distort_projection;
						cv::projectPoints(worldPoint, calib.R, calib.T, calib.proj_K, calib.proj_dist, distort_projection);		// ���e�p�ɘc�ݕ␳
						cv::drawChessboardCorners( proj_img, calib.checkerPattern, distort_projection, true );
						cv::namedWindow("proj_img", 0);
						Projection::MySetFullScrean(DISPLAY_NUMBER, "proj_img");
						cv::imshow("proj_img", proj_img);

						cv::waitKey(0);
					}
				}
			}
			break;

		case '6':
			if(calib.calib_flag)
			{
				std::cout << "3�����������c" << std::endl;

				// �O���C�R�[�h���e
				gc.code_projection();
				gc.make_thresh();
				gc.makeCorrespondence();

				// �Ή��_�̎擾
				std::vector<cv::Point2f> imagePoint;
				std::vector<cv::Point2f> projPoint;
				std::vector<cv::Point3f> reconstructPoint;
				gc.getCorrespondAllPoints(projPoint, imagePoint);

				// �Ή��_�̘c�ݏ���
				std::vector<cv::Point2f> undistort_imagePoint;
				std::vector<cv::Point2f> undistort_projPoint;
				cv::undistortPoints(imagePoint, undistort_imagePoint, calib.cam_K, calib.cam_dist);
				cv::undistortPoints(projPoint, undistort_projPoint, calib.proj_K, calib.proj_dist);
				for(int i=0; i<imagePoint.size(); ++i)
				{
					undistort_imagePoint[i].x = undistort_imagePoint[i].x * calib.cam_K.at<double>(0,0) + calib.cam_K.at<double>(0,2);
					undistort_imagePoint[i].y = undistort_imagePoint[i].y * calib.cam_K.at<double>(1,1) + calib.cam_K.at<double>(1,2);
					undistort_projPoint[i].x = undistort_projPoint[i].x * calib.proj_K.at<double>(0,0) + calib.proj_K.at<double>(0,2);
					undistort_projPoint[i].y = undistort_projPoint[i].y * calib.proj_K.at<double>(1,1) + calib.proj_K.at<double>(1,2);
				}

				// 3��������
				calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint);

				// �`��
				cv::Mat R = cv::Mat::eye(3,3,CV_64F);
				cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
				int key=0;
				cv::Point3d viewpoint(0.0,0.0,400.0);		// ���_�ʒu
				cv::Point3d lookatpoint(0.0,0.0,0.0);	// ��������
				const double step = 50;

				// �L�[�{�[�h����
				while(true)
				{
					// ��]�̍X�V
					double x=(lookatpoint.x-viewpoint.x);
					double y=(lookatpoint.y-viewpoint.y);
					double z=(lookatpoint.z-viewpoint.z);
					double pitch =asin(x/sqrt(x*x+z*z))/CV_PI*180.0;
					double yaw   =asin(-y/sqrt(y*y+z*z))/CV_PI*180.0;
					eular2rot(yaw, pitch, 0, R);
					// �ړ��̍X�V
					t.at<double>(0,0)=viewpoint.x;
					t.at<double>(1,0)=viewpoint.y;
					t.at<double>(2,0)=viewpoint.z;

					calib.pointCloudRender(reconstructPoint, imagePoint, cam2, std::string("viewer"), R, t);

					key = cv::waitKey(0);
					if(key=='w')
					{
						viewpoint.y+=step;
					}
					if(key=='s')
					{
						viewpoint.y-=step;
					}
					if(key=='a')
					{
						viewpoint.x+=step;
					}
					if(key=='d')
					{
						viewpoint.x-=step;
					}
					if(key=='z')
					{
						viewpoint.z+=step;
					}
					if(key=='x')
					{
						viewpoint.z-=step;
					}
					if(key=='q')
					{
						break;
					}
				}

			} else {
				std::cout << "�L�����u���[�V�����f�[�^������܂���" << std::endl;
			}

			break;


		case 'w':
			prjWhite = !prjWhite;
			break;

		default:
			exit(0);
			break;
		}
		printf("\n");
		cv::destroyAllWindows();
	}
}


// �I�C���[�p���s��ɕϊ�
void eular2rot(double yaw,double pitch, double roll, cv::Mat& dest)
{
    double theta = yaw/180.0*CV_PI;
    double pusai = pitch/180.0*CV_PI;
    double phi = roll/180.0*CV_PI;
 
    double datax[3][3] = {{1.0,0.0,0.0}, 
    {0.0,cos(theta),-sin(theta)}, 
    {0.0,sin(theta),cos(theta)}};
    double datay[3][3] = {{cos(pusai),0.0,sin(pusai)}, 
    {0.0,1.0,0.0}, 
    {-sin(pusai),0.0,cos(pusai)}};
    double dataz[3][3] = {{cos(phi),-sin(phi),0.0}, 
    {sin(phi),cos(phi),0.0}, 
    {0.0,0.0,1.0}};

    cv::Mat Rx(3,3,CV_64F,datax);
    cv::Mat Ry(3,3,CV_64F,datay);
    cv::Mat Rz(3,3,CV_64F,dataz);
    cv::Mat rr=Rz*Rx*Ry;

    rr.copyTo(dest);
}