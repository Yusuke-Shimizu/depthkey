#include <XnOS.h>
#include <glut.h>	// OpenGL

#include <iostream>		// ���o��
#include <sstream>		// ������X�g���[��
#include <fstream>
//#include <stdexcept>	// ��O.
//#include <direct.h>		// �f�B���N�g���w�b�_

//#include <math.h>			// ���w�n�w�b�_
#include <XnCppWrapper.h>	// OpenNI
//#include <xnFPSCalculator.h>// FPS����p
//#include "draw.h"

//---------------------------------------------------------------------------
// ���O���
//---------------------------------------------------------------------------
using namespace std;

//---------------------------------------------------------------------------
// ��`
//---------------------------------------------------------------------------
#define KINECT_IMAGE_WIDTH	640		// �摜�̕�(image(depth)MD.XRes)
#define KINECT_IMAGE_HEIGHT	480		// �摜�̍���(image(depth)MD.YRes)
#define KINECT_IMAGE_SIZE	307200	// �摜�̑傫��(640 * 480)
#define KINECT_MAX_DEPTH 10000		// �f�v�X�̍ő�l

#define BACK_FOWARD_NUM 10
#define HUMAN_FOWARD_NUM 10

//#define FOWARD_FLAG 1				// ��ɍ����摜��O�ɏo�����ǂ���

//---------------------------------------------------------------------------
// �R�[�h
//---------------------------------------------------------------------------
//----------------------------------------------------
// �����`��
//----------------------------------------------------
void glPrintString(const char *str, float x, float y, float z, void *font){
	int l = strlen(str);

	glRasterPos3f(x, y, z);

	for(int i = 0; i < l; i ++){
		glutBitmapCharacter(font,*str++);
	}
}

//----------------------------------------------------
// �����`��(ostringstream ver) (�G���[���ł� private���Ȃ񂿂��...)
//----------------------------------------------------
//void glPrintStringOss(ostringstream oss, float x, float y, float z, void *font){
//	glPrintString(oss.str().c_str(), x, y, z, font);
//	oss.str("");	// �o�b�t�@�̃N���A
//}

//----------------------------------------------------
// �|�C���g�N���E�h�̕`��
//----------------------------------------------------
void drawPointCloud(const XnRGB24Pixel*	_pImage, 
					const XnDepthPixel*	_pDepth, 
					XnPoint3D*			_pPoint, 
					const GLubyte		_forwardNum, 
					const int thresh)
{
	glBegin(GL_POINTS);		// �_��ł��Ă���
	{
		// �[���ɉ�����XnPoint3D�Ɋi�[
		const XnDepthPixel* pDepth = _pDepth;		// �f�v�X�f�[�^�̃|�C���^�擾
		const XnRGB24Pixel* pImage = _pImage;		// �C���[�W�f�[�^�̃|�C���^�擾
		XnPoint3D* point = _pPoint;

		for(XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; y ++){
			for(XnUInt x = 0; x < KINECT_IMAGE_WIDTH; x ++,pDepth ++, pImage ++, point ++){
				// �|�C���g�̐ݒ�
				point->X = x;
				point->Y = y;

				if(*pDepth == 0 || *pDepth > thresh){		// �[����0�Ȃ��ԉ��ɔz�u
					point->Z = KINECT_MAX_DEPTH - BACK_FOWARD_NUM;
				}else{
					point->Z = *pDepth;
				}

				// �`��
				glColor3ubv((GLubyte*)pImage);		// �F�̕\��
				glVertex3fv((GLfloat*)point);		// �_�̕\��
			}
		}
	}
	glEnd();	// �_��ł̂��I������
}

//----------------------------------------------------
// �l�����|�C���g�N���E�h�`��
//----------------------------------------------------
void drawPointCloudHuman(const XnRGB24Pixel*	_pImage, 
						 const XnDepthPixel*	_pDepth, 
						 const XnLabel*			_pLabel, 
						 XnPoint3D*				_pPoint, 
						 const GLubyte			_forwardNum)
{
	glBegin(GL_POINTS);		// �_��ł��Ă���
	{
		const XnDepthPixel* pDepth = _pDepth;		// �f�v�X�f�[�^�̃|�C���^�擾
		const XnRGB24Pixel* pImage = _pImage;
		const XnLabel* pLabel = _pLabel;			// �l���f�[�^�̃|�C���^�擾
		XnPoint3D* point = _pPoint;

		for(XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; y ++){
			for(XnUInt x = 0; x < KINECT_IMAGE_WIDTH; x ++, pDepth ++, pImage ++, pLabel ++, point ++){
				// �|�C���g�̐ݒ�
				point->X = x;
				point->Y = y;

				if(*pDepth == 0){		// �[����0�Ȃ��ԉ��ɔz�u
					point->Z = KINECT_MAX_DEPTH - HUMAN_FOWARD_NUM;
				}else{
#ifdef FOWARD_FLAG
					point->Z = 50;
#else 
					point->Z = *pDepth;
#endif
				}

				// �`��
				if(*pLabel != 0){
					glColor3ubv((GLubyte*)pImage);		// �F�̕\��
					glVertex3fv((GLfloat*)point);		// �_�̕\��
				}
			}
		}
	}
	glEnd();	// �_��ł̂��I������
}
