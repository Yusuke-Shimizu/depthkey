/******************************************

����

�w�b�_�i�N���X�j������������

�O���[�o���ϐ����炷

���[�U�N���X���

�g����w�i
13(A : 1.0, B : 2.0)
15(A : 1.0, B : 3.0)
16(A : 2.0, B : 2.0)

*******************************************/


//---------------------------------------------------------------------------
// �C���N���[�h
//---------------------------------------------------------------------------
#include <XnOS.h>
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <glut.h>	// OpenGL
#endif

#include <iostream>		// ���o��
#include <sstream>		// ������X�g���[��
#include <fstream>
#include <stdexcept>	// ��O.
#include <direct.h>		// �f�B���N�g���w�b�_

#include <math.h>			// ���w�n�w�b�_
#include <XnCppWrapper.h>	// OpenNI
#include <xnFPSCalculator.h>// FPS����p

#include "gl_screenshot.h"	// �X�N���[���V���b�g�w�b�_
#include "BMPLoader.h"		// bmp�ǂݍ��ݗp
#include "draw.h"

//---------------------------------------------------------------------------
// ���O���
//---------------------------------------------------------------------------
using namespace xn;
using std::cout;
using std::endl;

//---------------------------------------------------------------------------
// ��`
//---------------------------------------------------------------------------
//#define INIT_PATH "../Kinect/"
#define SAMPLE_XML_PATH "../Kinect/Data/SamplesConfig.xml"
#define BACK_IMAGE_PATH "../Kinect/Data/background/chromaBack16.bmp"
#define BACK_DEPTH_PATH "../Kinect/Data/background/chromaBack16.txt"
#define PLAYER_RECORDE_PATH "../Kinect/Data/player.oni"

#define OUT_IMAGE_PATH "../Kinect/bitmap/chromaBack17.bmp"
#define OUT_DEPTH_PATH "../Kinect/bitmap/chromaBack17.txt"
#define OUT_RECORDE_PATH "../Kinect/Data/record.oni"

#define GL_WIN_SIZE_X 1280	// �E�B���h�E�Y�̕�
#define GL_WIN_SIZE_Y 1024	// �E�B���h�E�Y�̍���

#define KINECT_IMAGE_WIDTH	640		// �摜�̕�(image(depth)MD.XRes)
#define KINECT_IMAGE_HEIGHT	480		// �摜�̍���(image(depth)MD.YRes)
#define KINECT_IMAGE_SIZE	307200	// �摜�̑傫��(640 * 480)
#define KINECT_MAX_DEPTH 10000		// �f�v�X�̍ő�l
#define KINECT_VISIBLE_DELTA 10		// ���p����

#define DISPLAY_MODE_OVERLAY		1				// �C���[�W�ƃf�v�X�̃I�[�o���C���[�h
#define DISPLAY_MODE_DEPTH			2				// �f�v�X���[�h
#define DISPLAY_MODE_IMAGE			3				// �C���[�W���[�h
#define DISPLAY_MODE_CHROMA			4				// �N���}�L�[���[�h
#define DISPLAY_MODE_POINT_CLOUD	5				// �|�C���g�N���E�h���[�h
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_IMAGE	// �f�t�H���g�̃��[�h

#define SHOW_MODE_NORMAL			1					// �C���[�W���[�h
#define SHOW_MODE_POINT_CLOUD		2					// �N���}�L�[���[�h
#define DEFAULT_SHOW_MODE			SHOW_MODE_NORMAL	// �f�t�H���g�̃��[�h

#define GL_CHAR_SIZE 20		// �����T�C�Y
#define DEBUG_X 15			// �f�o�b�O�̏����ʒuX
#define DEBUG_Y 25			// �f�o�b�O�̏����ʒuY

#define ZOOM_RATE 1			// �B�e���̐[�x�g�嗦

#define GL_GENERATE_MIPMAP_SGIS 0x8191	// �v����

//---------------------------------------------------------------------------
// �}�N��, inline
//---------------------------------------------------------------------------
/*#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}*/

// RGB�s�N�Z���̏�����
inline XnRGB24Pixel xnRGB24Pixel( int r, int g, int b ){
  XnRGB24Pixel pixel = { r, g, b };
  return pixel;
}

//---------------------------------------------------------------------------
// �x���̗}��
//---------------------------------------------------------------------------
//1>c:\users\shimizu\documents\visual studio 2008\projects\kinect\kinect\main.cpp(496) : warning C4996: 'xn::Context::InitFromXmlFile': Use other overload!
//1>        c:\program files (x86)\openni\include\xncppwrapper.h(5510) : 'xn::Context::InitFromXmlFile' �̐錾���m�F���Ă��������B
#pragma warning(disable:4996)

//---------------------------------------------------------------------------
// �O���[�o���ϐ�
//---------------------------------------------------------------------------
float g_pDepthHist[KINECT_MAX_DEPTH];
XnRGB24Pixel* g_pTexMap = NULL;						// �`�悷��Ƃ��́i�e�N�X�`���j���̃|�C���^
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;
unsigned int g_timeNum = 10000;						// �t�@�C���̐���
unsigned int g_chromaThresh = 3000;					// �N���}�L�[1�̃X���b�V�����h�l
GLfloat g_pointSize = 2.500001;						// �_�`��̓_�̑傫��
int g_currentWindowSizeX, g_currentWindowSizeY;		// ���݂̃E�B���h�E�T�C�Y

unsigned int g_nViewState = DEFAULT_DISPLAY_MODE;	// �`��̃��[�h
unsigned int g_nShowState = DEFAULT_SHOW_MODE;		// �\���̃��[�h
bool g_debugMode = true;							// �f�o�b�O���[�h
bool g_fullScreenMode = true;						// �t���X�N���[�����[�h
bool g_screenShotImageMode = false;					// �X�N���[���V���b�g�C���[�W���[�h
bool g_screenShotDepthMode = false;					// �X�N���[���V���b�g�f�v�X���[�h

GLdouble g_lokEyeX = 0, g_lokEyeY = 0, g_lokEyeZ = -1;				// ���_�����W
GLdouble g_lokDirX = 0, g_lokDirY = 0, g_lokDirZ = g_lokEyeZ - 1;	// ���_����W
XnPoint3D* g_pPoint = NULL;											// ���W������|�C���^

gl_screenshot g_glScreenShot;		// bmp�t�@�C���̏o��
BMPImage g_back;					// �w�i��bmp�t�@�C��
XnRGB24Pixel* g_pBackTex = NULL;	// �w�i�̃C���[�W�|�C���^
XnPoint3D* g_pBackPoint = NULL;		// �w�i�摜�̍��W������|�C���^
XnDepthPixel* g_pBackDepth = NULL;	// �w�i�摜�̐[��������|�C���^

XnFPSData g_xnFPS;					// FPS�v���p�f�[�^

Context g_context;
DepthGenerator g_depth;		// �f�v�X�W�F�l���[�^
ImageGenerator g_image;		// �C���[�W�W�F�l���[�^
UserGenerator g_user;		// ���[�U�W�F�l���[�^
DepthMetaData g_depthMD;	// �f�v�X�̎��f�[�^
ImageMetaData g_imageMD;	// �C���[�W�̎��f�[�^
SceneMetaData g_sceneMD;	// ���[�U�̎��f�[�^

Recorder g_recorder;	// �^��p���R�[�_�[
Player g_player;		// �Đ��p�v���C���[

// ���[�U�̐F
XnRGB24Pixel userColor[11] = {
	{255, 255, 255},
	{0, 0, 255},
	{0, 255, 0},
	{0, 255, 255},
	{255, 0, 0},
	{255, 0, 255},
	{255, 255, 0},
	{0, 0, 127},
	{0, 127, 0},
	{127, 0, 0},
	{0, 127, 255}
};

//----------------------------------------------------
// �֐��v���g�^�C�v�i��ɌĂяo���֐����ƈ����̐錾�j
//----------------------------------------------------
void xnInit(void);
void glInit(int *argcp, char **argv);
void backInit(void);
void playerInit(void);
void glutIdle (void);
void glutDisplay (void);
void glutResize (int w, int h);
void glutKeyboard (unsigned char key, int x, int y);
void glutMouse(int button, int state, int _x, int _y);
void glutCallback(void);
void glDebug(void);
XnStatus setRecorder(Recorder recorder, XnStatus rc);
void setDepthHistgram(const xn::DepthGenerator& depth, const xn::DepthMetaData& depthMD, float _pDepthHist[]);
void setTexture(void);
void drawImage(void);
void XN_CALLBACK_TYPE UserDetected(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserLost(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void errorCheck(XnStatus status, char* what);


//---------------------------------------------------------------------------
// �R�[�h
//---------------------------------------------------------------------------
//----------------------------------------------------
// ���C���֐�
//----------------------------------------------------
int main(int argc, char* argv[]){
	// �e�평����
	xnInit();													// OpenNI�֘A�̏�����
	glInit(&argc, argv);
	backInit();
	
	// Per frame code is in glutDisplay
	glutMainLoop();

	return 0;
}

//----------------------------------------------------
// OpenNI�֘A�̏�����
//----------------------------------------------------
void xnInit(void){
	XnStatus rc;

	EnumerationErrors errors;
	rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT){
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		exit(1);
	}else if (rc != XN_STATUS_OK){
		printf("Open failed: %s\n", xnGetStatusString(rc));
		exit(1);
	}
	
	//playerInit();

	rc = xnFPSInit(&g_xnFPS, 180);	// FPS�̏�����
	//CHECK_RC(rc, "FPS Init");

	// �f�v�X�E�C���[�W�E���[�U�W�F�l���[�^�̍쐬
	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	errorCheck(rc, "g_depth");		// �G���[�`�F�b�N
	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	errorCheck(rc, "g_image");
	rc = g_context.FindExistingNode(XN_NODE_TYPE_USER, g_user);
	//rc = g_user.Create(g_context);
	errorCheck(rc, "g_user");

	// ���[�U�[���o�@�\���T�|�[�g���Ă��邩�m�F
	if (!g_user.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		//throw std::runtime_error("���[�U�[���o���T�|�[�g���Ă܂���");
		cout << "���[�U�[���o���T�|�[�g���Ă܂���" << endl;
		exit(1);
	}

	// ���R�[�_�[�̐ݒ�
	//rc = setRecorder(g_recorder, rc);

	// ���[�U�R�[���o�b�N�̓o�^
	XnCallbackHandle userCallbacks;
	g_user.RegisterUserCallbacks(UserDetected, UserLost, NULL, userCallbacks);

	// �f�v�X�E�C���[�W�E���[�U�f�[�^�̎擾
	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);
	g_user.GetUserPixels(0, g_sceneMD);

	// Hybrid mode isn't supported in this sample
	// �C���[�W�ƃf�v�X�̑傫�����Ⴄ�ƃG���[
	if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes()){
		printf ("The device depth and image resolution must be equal!\n");
		exit(1);
	}

	// RGB is the only image format supported.
	// �t�H�[�}�b�g�̊m�F
	if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24){
		printf("The device image format must be RGB24\n");
		exit(1);
	}

	// Texture map init
	// �t���X�N���[����ʂ̑傫������
	g_nTexMapX = (((unsigned short)(g_depthMD.FullXRes() - 1) / 512) + 1) * 512;	// �傫���ɂ����512�̔{���ɒ���(1024)
	g_nTexMapY = (((unsigned short)(g_depthMD.FullYRes() - 1) / 512) + 1) * 512;	// 512
	g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));	// �X�N���[���̑傫�����̐F���̗e�ʂ��m��

	// ���W�|�C���^�̏�����
	g_pPoint = (XnPoint3D*)malloc(KINECT_IMAGE_SIZE * sizeof(XnPoint3D));			// ���W������|�C���^���쐬
	g_pBackTex = (XnRGB24Pixel*)malloc(KINECT_IMAGE_SIZE * sizeof(XnRGB24Pixel));	// �w�i�摜������|�C���^���쐬
	g_pBackPoint = (XnPoint3D*)malloc(KINECT_IMAGE_SIZE * sizeof(XnPoint3D));		// �w�i���W������|�C���^���쐬
	g_pBackDepth = (XnDepthPixel*)malloc(KINECT_IMAGE_SIZE * sizeof(XnDepthPixel));		// �w�i���W������|�C���^���쐬
}

//----------------------------------------------------
// openGL�̏�����
//----------------------------------------------------
void glInit(int *argcp, char **argv){
	// OpenGL init
	glutInit(argcp, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);	//�f�B�X�v���C���[�h�̎w��
	glutInitWindowSize(KINECT_IMAGE_WIDTH, KINECT_IMAGE_HEIGHT);			//�E�B���h�E�T�C�Y�̎w��
	glutCreateWindow ("Kinect Viewer");							//�E�B���h�E�̍쐬
	//glutFullScreen();											// �t���X�N���[���ɂ���
	//glutSetCursor(GLUT_CURSOR_NONE);							// �J�[�\���̃C���[�W������

	glutCallback();		// �R�[���o�b�N
}

//----------------------------------------------------
// �w�i�̏�����
//----------------------------------------------------
void backInit(void){
	// �w�i�̃C���[�W�f�[�^������
	g_back.Load(BACK_IMAGE_PATH);		// �w�i�摜�̏�����

	// BMPImage(g_back) -> XnRGB24Pixel(g_pBackTex)
	XnRGB24Pixel* pBack = g_pBackTex;													// �w�i�摜������|�C���^
	GLuint g_backWidth = g_back.GetWidth();												// �w�i�̉����̑傫��
	GLubyte* pBackData = g_back.GetData() + g_back.GetImageSize() - 3 * g_backWidth;	// �w�i�̃|�C���^�擾(�Ōォ�猩�Ă���)

	for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){	// 480
		for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pBack){	// 640
			// �[����0��臒l�ȏ�Ȃ�w�i�摜��`��i臒l�ȉ��Ȃ炻�̕������c���j
			pBack->nRed		= *pBackData;
			pBack->nGreen	= *(pBackData + 1);
			pBack->nBlue	= *(pBackData + 2);

			pBackData += 3;		// RGB��3���i�߂�
		}

		pBackData -= 2 * 3 * g_backWidth;	// ��s��RGB3�����i2�s�j���߂�
	}

	// �w�i�̃f�v�X�f�[�^(g_pBackData)������
	//g_backDepthData.open();	// �w�i�̐[���f�[�^���J��
	ifstream backDepthData(BACK_DEPTH_PATH);			// �w�i�̐[���f�[�^
	
	//XnPoint3D* pBackPoint = g_pBackPoint;
	XnDepthPixel* pBackDepth = g_pBackDepth;

	// �w�i�̍��W�l���擾
	for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; y ++){
		char c[10];
		for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; x ++, /*pBackPoint ++,*/ pBackDepth ++){
			//pBackPoint->X = x;
			//pBackPoint->Y = y;
			//pBackPoint->Z = atoi(c);
			backDepthData.getline(c, 10, ',');
			*pBackDepth = atoi(c);
			//cout << *pBackDepth << endl;
		}
		backDepthData.getline(c, 10);
	}

}

//----------------------------------------------------
// �v���C���[�̏�����
//----------------------------------------------------
void playerInit(void){
	XnStatus rc;

	rc = g_context.Init();
	rc = g_context.OpenFileRecording(PLAYER_RECORDE_PATH, g_player);
	errorCheck(rc, "OpenFileRecording");		// �G���[�`�F�b�N

	rc = g_context.FindExistingNode(XN_NODE_TYPE_PLAYER, g_player);
	errorCheck(rc, "FindExistingNode_player");

	// �T�|�[�g�����t�H�[�}�b�g�̎擾
	cout << "SupportedFormat:" <<
		g_player.GetSupportedFormat() << endl;

	// �Đ����x�̎擾
	cout << "PlaybackSpeed:" << g_player.GetPlaybackSpeed() << endl;
}

//---------------------------------------------------------------------------
// �R�[���o�b�N�֐�
//---------------------------------------------------------------------------
//----------------------------------------------------
// �A�C�h�����̏���
//----------------------------------------------------
void glutIdle (void){
	// Display the frame
	glutPostRedisplay();
}

//----------------------------------------------------
// �`�揈��
//----------------------------------------------------
void glutDisplay (void){
	xnFPSMarkFrame(&g_xnFPS);		// FPS�̌v���J�n�H

	XnStatus rc = XN_STATUS_OK;

	// �X�V���ꂽ�m�[�h��҂�(�ǂ�ł�����)
	rc = g_context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK){
		printf("Read failed: %s\n", xnGetStatusString(rc));
		printf("test\n");
		return;
	}

	// �C���[�W�E�f�v�X�E���[�U�̃f�[�^���擾
	g_image.GetMetaData(g_imageMD);
	g_depth.GetMetaData(g_depthMD);
	g_user.GetUserPixels(0, g_sceneMD);

	// �J���[�E�f�v�X�o�b�t�@���N���A
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// �ݒ�
	setDepthHistgram(g_depth, g_depthMD, g_pDepthHist);	// �q�X�g�O�����̌v�Z�E�쐬
	setTexture();	// �e�N�X�`���ݒ�

	// �`��
	drawImage();	// �C���[�W�f�[�^�̕`��

	// �f�o�b�O���[�h�̕����͕`��̍s��Ɗu��
	glMatrixMode(GL_PROJECTION);								// �ˉe�ϊ��̍s��̐ݒ�
	//glLoadIdentity();											// �X�^�b�N�̃N���A
	glMatrixMode(GL_MODELVIEW);								// ���f���r���[�ϊ��̍s��̐ݒ�
	glLoadIdentity();
	if(g_debugMode) glDebug();								// �f�o�b�O���[�h

	// ��x�����X�N���[���V���b�g���Ƃ�
	if(g_screenShotImageMode){
		ostringstream fname;
		fname	<< OUT_IMAGE_PATH ;//�o�̓t�@�C����
		std::string name = fname.str();
		g_glScreenShot.screenshot(name.c_str(), 24);

		g_screenShotImageMode = !g_screenShotImageMode;	// �g�O��
	}

	// ��x�����[���f�[�^���擾����
	if(g_screenShotDepthMode){
		ofstream ofs(OUT_DEPTH_PATH);
		const XnDepthPixel* pDepth = g_depthMD.Data();

		for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; y ++){
			for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; x ++, pDepth ++){
				if(*pDepth < 2000){
					ofs << (int)((*pDepth) * 2) << ',';
				}else{
					ofs << (*pDepth) << ',';
				}
			}
			ofs << endl;
		}

		g_screenShotDepthMode = !g_screenShotDepthMode;	// �g�O��
	}

	// Swap the OpenGL display buffers
	glutSwapBuffers();
}

//----------------------------------------------------
// ���T�C�Y����
//----------------------------------------------------
void glutResize (int w, int h){
	// �ύX��̃E�B���h�E�T�C�Y���擾
	g_currentWindowSizeX = w;
	g_currentWindowSizeY = h;

	// �E�B���h�E�S�̂��r���[�|�[�g�ɂ���
	glViewport(0, 0, w, h);

	// �|�C���g�T�C�Y�̕ύX
	//g_pointSize = ;
	GLfloat tmp1 = (w * w * 6.10364 / 10000000.0) + (w * 0.00117187);
	GLfloat tmp2 = (h * h * 6.5824 / 10000000.0) + (h * 0.00176738);

	// �_���傫������ݒ肷��
	//if(tmp1 > tmp2){
		g_pointSize = tmp1;
	//}else{
	//	g_pointSize = tmp2;
	//}

	cout << "resize(" << w << ", " << h << "), point = " << g_pointSize << endl;

	// �ϊ��s��̏�����
	//glLoadIdentity();
}

//----------------------------------------------------
// �L�[�{�[�h����
//----------------------------------------------------
void glutKeyboard (unsigned char key, int x, int y){
	switch (key){
		case '1':	// �I�[�o�[���C���[�h
			g_nViewState = DISPLAY_MODE_OVERLAY;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// �C���[�W�ƃf�v�X�̂���𖳂���
			break;
		case '2':	// �f�v�X���[�h
			g_nViewState = DISPLAY_MODE_DEPTH;
			g_depth.GetAlternativeViewPointCap().ResetViewPoint();		// �f�v�X�̑傫�������ɖ߂��H
			break;
		case '3':	// �C���[�W���[�h
			g_nViewState = DISPLAY_MODE_IMAGE;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// �C���[�W�ƃf�v�X�̂���𖳂���
			break;
		case '4':	// �N���}�L�[���[�h
			g_nViewState = DISPLAY_MODE_CHROMA;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// �C���[�W�ƃf�v�X�̂���𖳂���
			break;
		case '5':	// �|�C���g�N���E�h���[�h
			g_nViewState = DISPLAY_MODE_POINT_CLOUD;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// �C���[�W�ƃf�v�X�̂���𖳂���
			break;
			
		case 'm':	// ���A�Ō��ցI�I�i���̂��d���Ȃ�j��
			g_context.SetGlobalMirror(!g_context.GetGlobalMirror());	// �~���[�����O
			break;
		case 'd':
			g_debugMode = !g_debugMode;									// �f�o�b�O���[�h�̃I���I�t
			break;

		case 'f':
			g_fullScreenMode = !g_fullScreenMode;	// �t���X�N���[�����[�h�̐؂�ւ��i�g�O���X�C�b�`�j

			if(g_fullScreenMode){					// �t���X�N���[�����[�h
				glutFullScreen();
			}else{									// �E�B���h�E���[�h
				glutPositionWindow(100, 100);
				glutReshapeWindow(KINECT_IMAGE_WIDTH, KINECT_IMAGE_HEIGHT);
			}
			break;

		case 's':	// �X�N���[���V���b�g���B��i�[�����B��j
			g_screenShotDepthMode = true;
		case 'S':	// �X�N���[���V���b�g���B��i�[���͎B��Ȃ��j
			g_screenShotImageMode = true;
			break;

		case 'R':	// ���R�[�h�X�g�b�v
			g_recorder.RemoveNodeFromRecording(g_image);
			g_recorder.RemoveNodeFromRecording(g_depth);

			cout << "recording stop!" << endl;
			break;

		// 臒l�̑���
		case 't': g_chromaThresh += 10; break;
		case 'T': g_chromaThresh -= 10; break;

		//case 'p':
		//	g_pointSize += 0.000001;
		//	break;
		//case 'P':
		//	g_pointSize -= 0.000001;
		//	break;

		// ���_�ړ�
		case 'x': g_lokEyeX ++; g_lokDirX ++; break;
		case 'X': g_lokEyeX --; g_lokDirX --; break;
		case 'y': g_lokEyeY ++; g_lokDirY ++; break;
		case 'Y': g_lokEyeY --; g_lokDirY --; break;
		case 'z': g_lokEyeZ += 1; g_lokDirZ += 1; break;
		case 'Z': g_lokEyeZ -= 1; g_lokDirZ -= 1; break;

		// �����I��
		case 27:	// Esc�{�^��
		case 'q':
		case 'Q':
			exit (1);
	}
}

//----------------------------------------------------
// �}�E�X�̃N���b�N����
//----------------------------------------------------
void glutMouse(int button, int state, int _x, int _y){
	int x = _x, y = _y;
	XnPoint3D pt[2] = {{0,0,0},{0,0,0}};

	// �T�C�Y���Ⴄ�ꍇ�C680*480�ɕW��������
	if(!(g_currentWindowSizeX == KINECT_IMAGE_WIDTH && g_currentWindowSizeY == KINECT_IMAGE_HEIGHT)){
		x = 640 * _x / g_currentWindowSizeX;
		y = 480 * _y / g_currentWindowSizeY;
	}

	if(state == GLUT_DOWN){
		if(button == GLUT_LEFT_BUTTON){			// ���N���b�N
			cout << "click! (" << _x << ", " << _y << ")->(" << x << ", " << y << "), depth = " << *(g_depthMD.Data() + y * KINECT_IMAGE_WIDTH + x) << endl;
			pt[0].X = _x;
			pt[0].Y = _y;
			pt[0].Z = *(g_depthMD.Data() + y * KINECT_IMAGE_WIDTH + x);
			//cout << "(_x, _y) -> (x, y) = (" << _x << ", " << _y << ") -> (" << x << ", " << y << ")" << endl;

			g_depth.ConvertProjectiveToRealWorld(2, pt, pt);

			cout << "change pt[0] => (" << pt[0].X << ", " << pt[0].Y << ", " << pt[0].Z << ")" << endl;
		}else if(button == GLUT_RIGHT_BUTTON){	// �E�N���b�N
			cout << "click! back = (" << x << ", " << y << ") depth = " << *(g_pBackDepth + y * KINECT_IMAGE_WIDTH + x) << endl;
			pt[1].X = _x;
			pt[1].Y = _y;
			pt[1].Z = *(g_pBackDepth + y * KINECT_IMAGE_WIDTH + x);

			g_depth.ConvertProjectiveToRealWorld(2, pt, pt);

			cout << "change pt[1] => (" << pt[1].X << ", " << pt[1].Y << ", " << pt[1].Z << ")" << endl;
		}

		//g_depth.ConvertProjectiveToRealWorld(2, pt, pt);

		//cout << "change pt[0] => (" << pt[0].X << ", " << pt[0].Y << ", " << pt[0].Z << ")" << endl;
		//cout << "change pt[1] => (" << pt[1].X << ", " << pt[1].Y << ", " << pt[1].Z << ")" << endl;
	}
}
//----------------------------------------------------
// �R�[���o�b�N�֐��̂܂Ƃ�
//----------------------------------------------------
void glutCallback(void){
	glutKeyboardFunc(glutKeyboard);		// �L�[�{�[�h���͎��ɌĂяo�����֐����w�肷��i�֐����FglutKeyboard�j
	glutMouseFunc(glutMouse);			// 
	glutDisplayFunc(glutDisplay);		// �`�掞�ɌĂяo�����֐����w�肷��i�֐����FglutDisplay�j
	glutReshapeFunc(glutResize);		// �E�B���h�E�T�C�Y�ύX���ɌĂяo�����֐��̎w��
	glutIdleFunc(glutIdle);				// �v���O�����A�C�h����Ԏ��ɌĂяo�����֐�
}

//----------------------------------------------------
// ���R�[�_�[�̐ݒ�
//----------------------------------------------------
XnStatus setRecorder(Recorder recorder, XnStatus rc){
	//XnStatus rc;

	// ���R�[�_�[�̍쐬
	rc = recorder.Create(g_context);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}

	// �L�^�ݒ�
	rc = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, OUT_RECORDE_PATH);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}
    
	// �C���[�W���L�^�Ώۂɒǉ�
	rc = recorder.AddNodeToRecording(g_image, XN_CODEC_JPEG);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}
    
	// �f�v�X���L�^�Ώۂɒǉ�
	rc = recorder.AddNodeToRecording(g_depth, XN_CODEC_UNCOMPRESSED);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		std::cout << __LINE__ << std::endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}
    
	// �L�^�J�n(WaitOneUpdateAll�̃^�C�~���O�ŋL�^�����)
	rc = recorder.Record();
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}

	cout << "record set ok!" << endl;

	return rc;
}

//----------------------------------------------------
// �f�o�b�O�֐�
//----------------------------------------------------
void glDebug(void){
	int deb_x = DEBUG_X, deb_y = DEBUG_Y;
	ostringstream oss;

	glColor3f(0, 0, 1);

	glPrintString("DEBUG MODE", deb_x, deb_y);

	oss << "VIEW MODE -> " << g_nViewState;									// view���[�h�̕\��
	glPrintString(oss.str().c_str(), deb_x += GL_CHAR_SIZE, deb_y += GL_CHAR_SIZE * 2);
	oss.str("");	// �o�b�t�@�̃N���A
	oss << "windowSize -> (" << g_currentWindowSizeX << ", " << g_currentWindowSizeY << ")";	// ���݂̃E�B���h�E�T�C�Y�̕\��
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE);
	oss.str("");

	oss << "FPS : " << xnFPSCalc(&g_xnFPS);									// FPS�̕\��
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE * 2);
	oss.str("");
	oss << "threshold -> " << g_chromaThresh;								// �X���b�V�����h�̕\��
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE);
	oss.str("");
	oss << "Look (" << g_lokEyeX << "," << g_lokEyeY << "," << g_lokEyeZ << ") -> (" 
		<< g_lokDirX << "," << g_lokDirY << "," << g_lokDirZ << ")";		// ���_�̕\��
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE);	
	oss.str("");

	oss << "g_pointSize = " << g_pointSize;									// �_�̑傫���̕\��
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE * 2);
	oss.str("");
}

//----------------------------------------------------
// �q�X�g�O�����쐬�֐�
//----------------------------------------------------
void setDepthHistgram(const xn::DepthGenerator& depth, const xn::DepthMetaData& depthMD, float _pDepthHist[]){
	xnOSMemSet(_pDepthHist, 0, KINECT_MAX_DEPTH * sizeof(float));	// g_pDepthHist�̑S�Ă�0����

	unsigned int points = 0;
	const XnDepthPixel* pDepth = depthMD.Data();
	for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y) {
		for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pDepth) {
			if (*pDepth != 0) {
				_pDepthHist[*pDepth] ++;
				points ++;
			}
		}
	}

	for (int i = 1; i < KINECT_MAX_DEPTH; ++ i) {
		_pDepthHist[i] += _pDepthHist[i - 1];
	}

	if ( points != 0) {
		for (int i = 1; i < KINECT_MAX_DEPTH; ++ i) {
			_pDepthHist[i] = (unsigned int)(256 * (1.0f - (_pDepthHist[i] / points)));
   		}
	}
}


//----------------------------------------------------
// �e�N�X�`���̐ݒ�
//----------------------------------------------------
void setTexture(void){
	xnOSMemSet(g_pTexMap, 0, g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));	// g_pTexMap�̑S�Ă�0����

	// �`�惂�[�h1��3
	if (g_nViewState == DISPLAY_MODE_OVERLAY || g_nViewState == DISPLAY_MODE_IMAGE){
		const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();	// g_imageMD�̃|�C���^�擾(�摜�f�[�^�擾)
		XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

		for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){
			const XnRGB24Pixel* pImage = pImageRow;
			XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();

			for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pImage, ++ pTex){
				*pTex = *pImage;
			}

			pImageRow += g_imageMD.XRes();
			pTexRow += g_nTexMapX;
		}
	}

	// �`�惂�[�h1��2
	if (g_nViewState == DISPLAY_MODE_OVERLAY || g_nViewState == DISPLAY_MODE_DEPTH){
		const XnDepthPixel* pDepthRow = g_depthMD.Data();
		XnRGB24Pixel* pTexRow = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;
		const XnLabel* pLabel = g_sceneMD.Data();

		for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){
			const XnDepthPixel* pDepth = pDepthRow;
			XnRGB24Pixel* pTex = pTexRow + g_depthMD.XOffset();

			for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pDepth, ++ pTex, ++ pLabel){
				int nHistValue = g_pDepthHist[*pDepth];

				if(*pLabel){		// �l���Ȃ�
					*pTex = userColor[*pLabel];
				}else if (*pDepth != 0){
					if(*pDepth < 1000){
						*pTex = xnRGB24Pixel(nHistValue, 0, 0);		// red
					}else if(*pDepth < 2000){
						*pTex = xnRGB24Pixel(0, nHistValue, 0);		// green
					}else if(*pDepth < 3000){
						*pTex = xnRGB24Pixel(0, 0, nHistValue);		// blue
					}else if(*pDepth < 4000){
						*pTex = xnRGB24Pixel(nHistValue, nHistValue, 0);	// ���F
					}else if(*pDepth < 5000){
						*pTex = xnRGB24Pixel(0, nHistValue, nHistValue);	// yellow
					}else{
						*pTex = xnRGB24Pixel(nHistValue, 0, nHistValue);	// ��
					}
				}
			}

			pDepthRow += g_depthMD.XRes();
			pTexRow += g_nTexMapX;
		}
	}

	// �`�惂�[�h4
	//if (g_nViewState == DISPLAY_MODE_CHROMA){
	//	// �C���[�W�f�[�^(�J�����f��)�\��t��
	//	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();	// g_imageMD�̃|�C���^�擾(�摜�f�[�^�擾)
	//	XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

	//	for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){	// 480
	//		const XnRGB24Pixel* pImage = pImageRow;
	//		XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();

	//		for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pImage, ++ pTex){	// 640
	//			*pTex = *pImage;
	//		}

	//		pImageRow += g_imageMD.XRes();
	//		pTexRow += g_nTexMapX;
	//	}

	//	// �f�v�X�f�[�^��p�����l�������o�� + �w�i����
	//	const XnDepthPixel* pDepthRow = g_depthMD.Data();		// �f�v�X�f�[�^�̃|�C���^�擾
	//	pTexRow = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;
	//	GLuint g_backWidth = g_back.GetWidth();						// �w�i�̉����̑傫��
	//	GLubyte* pBackData = g_back.GetData() + g_back.GetImageSize() - 3 * g_backWidth;	// �w�i�̃|�C���^�擾(�Ōォ�猩�Ă���)

	//	for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){	// 480
	//		const XnDepthPixel* pDepth = pDepthRow;			// �f�v�X�f�[�^�̃|�C���^�擾
	//		XnRGB24Pixel* pTex = pTexRow + g_depthMD.XOffset();

	//		for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pDepth, ++ pTex){	// 640
	//			// �[����0��臒l�ȏ�Ȃ�w�i�摜��`��i臒l�ȉ��Ȃ炻�̕������c���j
	//			if (*pDepth == 0 || *pDepth >= g_chromaThresh){
	//				pTex->nRed		= *pBackData;
	//				pTex->nGreen	= *(pBackData + 1);
	//				pTex->nBlue		= *(pBackData + 2);
	//			}

	//			pBackData += 3;
	//		}

	//		pDepthRow += g_depthMD.XRes();
	//		pTexRow += g_nTexMapX;
	//		pBackData -= 2 * 3 * g_backWidth;
	//	}
	//}
}

//----------------------------------------------------
// �C���[�W�`��
//----------------------------------------------------
void drawImage(void){
	switch(g_nViewState){
		case DISPLAY_MODE_OVERLAY:		// �m�[�}���`�惂�[�h
		case DISPLAY_MODE_DEPTH:
		case DISPLAY_MODE_IMAGE:

			glMatrixMode(GL_PROJECTION);								// �ˉe�ϊ��̍s��̐ݒ�
			glLoadIdentity();											// �X�^�b�N�̃N���A
			gluOrtho2D(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0);	// ���[���h���W�n�𐳋K���f�o�C�X���W�n�ɕ��s���e(left, right, buttom, top, near, far)
															// �����s���e���鎖�ɂ���āC�|�C���g�N���E�h�����ʂɓ��e�ł��C�N���}�L�[�ɍœK
															// Kinect�̋����͖�500�`9000�܂Ŏg����(�ݒ��10000)
			glMatrixMode(GL_MODELVIEW);						// ���f���r���[�ϊ��̍s��̐ݒ�
			glLoadIdentity();

			glEnable(GL_TEXTURE_2D);	// �e�N�X�`���}�b�s���O�̗L����

			// �e�N�X�`���p�����[�^�̐ݒ�ƒ�`
			glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, g_pTexMap);	// �C���[�W�f�[�^�\��t��

			// Display the OpenGL texture map
			glColor4f(1,1,1,1);

			// �C���[�W�f�[�^�̓\��t��
			glBegin(GL_QUADS);		// �l�p�`�̕`����s��
			{
				int nXRes = g_depthMD.FullXRes();
				int nYRes = g_depthMD.FullYRes();

				// ����
				glTexCoord2f(0, 0);
				glVertex2f(0, 0);	// ���W�w��
				// �E��
				glTexCoord2f((float)nXRes/(float)g_nTexMapX, 0);
				glVertex2f(GL_WIN_SIZE_X, 0);	// ���W�w��
				// �E��
				glTexCoord2f((float)nXRes/(float)g_nTexMapX, (float)nYRes/(float)g_nTexMapY);
				glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);	// ���W�w��
				// ����
				glTexCoord2f(0, (float)nYRes/(float)g_nTexMapY);
				glVertex2f(0, GL_WIN_SIZE_Y);	// ���W�w��
			}
			glEnd();

			glDisable(GL_TEXTURE_2D);	// �e�N�X�`���}�b�s���O�̖�����

			break;

		case DISPLAY_MODE_CHROMA:		// �|�C���g�N���E�h�`�惂�[�h
		case DISPLAY_MODE_POINT_CLOUD:

			// ���e�ϊ�
			glMatrixMode(GL_PROJECTION);								// �ˉe�ϊ��̍s��̐ݒ�
			glLoadIdentity();											// �X�^�b�N�̃N���A
			glOrtho(0, KINECT_IMAGE_WIDTH, 
				KINECT_IMAGE_HEIGHT, 0, 
				-1.0, -KINECT_MAX_DEPTH - KINECT_VISIBLE_DELTA);	// ���[���h���W�n�𐳋K���f�o�C�X���W�n�ɕ��s���e(left, right, buttom, top, near, far)
																	// �����s���e���鎖�ɂ���āC�|�C���g�N���E�h�����ʂɓ��e�ł��C�N���}�L�[�ɍœK
																	// Kinect�̋����͖�500�`9000�܂Ŏg����(�ݒ��10000)
			// ����ϊ�
			gluLookAt(
				g_lokEyeX, g_lokEyeY, g_lokEyeZ,	// ���_�̈ʒu(�����ʒu�F(0,0,-1))
				g_lokDirX, g_lokDirY, g_lokDirZ,	// ���_��̈ʒu(�����ʒu�F(0,0,-2))
				0.0, 1.0, 0.0);						// ����
	 
			// ���f�����O�ϊ�
			glMatrixMode(GL_MODELVIEW);								// ���f���r���[�ϊ��̍s��̐ݒ�
			glLoadIdentity();										// �X�^�b�N�̃N���A

			glEnable(GL_DEPTH_TEST);	// �A�ʏ����̗L����

			// �|�C���g�N���E�h�\��
			glPointSize(g_pointSize);			// �_�̃T�C�Y
			drawPointCloud(g_pBackTex, g_pBackDepth, g_pPoint);					//�w�i�摜�\��
			//drawPointCloud(g_imageMD.RGB24Data(), g_depthMD.Data(), 10, g_chromaThresh);	// �l�������o��(������臒l)
			drawPointCloudHuman(g_imageMD.RGB24Data(), g_depthMD.Data(), g_sceneMD.Data(), g_pPoint);	// �l�������o��(�������̂����o)

			glDisable(GL_DEPTH_TEST);	// �A�ʏ����̖�����
			break;

	}
}

//----------------------------------------------------
// ���[�U�[���o
//----------------------------------------------------
void XN_CALLBACK_TYPE UserDetected(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	cout << "���[�U�[���o:" << nId << " " << generator.GetNumberOfUsers() << "�l��" << endl;
}

//----------------------------------------------------
// ���[�U�[����
//----------------------------------------------------
void XN_CALLBACK_TYPE UserLost(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	cout << "���[�U�[����:" << nId << endl;
}

//----------------------------------------------------
// �G���[�`�F�b�N
//----------------------------------------------------
void errorCheck(XnStatus status, char* what){
	if (status != XN_STATUS_OK) {
		cout << "error! : " << what << endl;
		//throw std::runtime_error(xnGetStatusString(status));
		exit(1);
	}	
}
