#include <XnOS.h>
#include <glut.h>	// OpenGL

#include <iostream>		// 入出力
#include <sstream>		// 文字列ストリーム
#include <fstream>
//#include <stdexcept>	// 例外.
//#include <direct.h>		// ディレクトリヘッダ

//#include <math.h>			// 数学系ヘッダ
#include <XnCppWrapper.h>	// OpenNI
//#include <xnFPSCalculator.h>// FPS測定用
//#include "draw.h"

//---------------------------------------------------------------------------
// 名前空間
//---------------------------------------------------------------------------
using namespace std;

//---------------------------------------------------------------------------
// 定義
//---------------------------------------------------------------------------
#define KINECT_IMAGE_WIDTH	640		// 画像の幅(image(depth)MD.XRes)
#define KINECT_IMAGE_HEIGHT	480		// 画像の高さ(image(depth)MD.YRes)
#define KINECT_IMAGE_SIZE	307200	// 画像の大きさ(640 * 480)
#define KINECT_MAX_DEPTH 10000		// デプスの最大値

#define BACK_FOWARD_NUM 10
#define HUMAN_FOWARD_NUM 10

//#define FOWARD_FLAG 1				// 常に合成画像を前に出すかどうか

//---------------------------------------------------------------------------
// コード
//---------------------------------------------------------------------------
//----------------------------------------------------
// 文字描画
//----------------------------------------------------
void glPrintString(const char *str, float x, float y, float z, void *font){
	int l = strlen(str);

	glRasterPos3f(x, y, z);

	for(int i = 0; i < l; i ++){
		glutBitmapCharacter(font,*str++);
	}
}

//----------------------------------------------------
// 文字描画(ostringstream ver) (エラーがでる privateがなんちゃら...)
//----------------------------------------------------
//void glPrintStringOss(ostringstream oss, float x, float y, float z, void *font){
//	glPrintString(oss.str().c_str(), x, y, z, font);
//	oss.str("");	// バッファのクリア
//}

//----------------------------------------------------
// ポイントクラウドの描画
//----------------------------------------------------
void drawPointCloud(const XnRGB24Pixel*	_pImage, 
					const XnDepthPixel*	_pDepth, 
					XnPoint3D*			_pPoint, 
					const GLubyte		_forwardNum, 
					const int thresh)
{
	glBegin(GL_POINTS);		// 点を打っていく
	{
		// 深さに応じてXnPoint3Dに格納
		const XnDepthPixel* pDepth = _pDepth;		// デプスデータのポインタ取得
		const XnRGB24Pixel* pImage = _pImage;		// イメージデータのポインタ取得
		XnPoint3D* point = _pPoint;

		for(XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; y ++){
			for(XnUInt x = 0; x < KINECT_IMAGE_WIDTH; x ++,pDepth ++, pImage ++, point ++){
				// ポイントの設定
				point->X = x;
				point->Y = y;

				if(*pDepth == 0 || *pDepth > thresh){		// 深さが0なら一番奥に配置
					point->Z = KINECT_MAX_DEPTH - BACK_FOWARD_NUM;
				}else{
					point->Z = *pDepth;
				}

				// 描画
				glColor3ubv((GLubyte*)pImage);		// 色の表示
				glVertex3fv((GLfloat*)point);		// 点の表示
			}
		}
	}
	glEnd();	// 点を打つのを終了する
}

//----------------------------------------------------
// 人物をポイントクラウド描画
//----------------------------------------------------
void drawPointCloudHuman(const XnRGB24Pixel*	_pImage, 
						 const XnDepthPixel*	_pDepth, 
						 const XnLabel*			_pLabel, 
						 XnPoint3D*				_pPoint, 
						 const GLubyte			_forwardNum)
{
	glBegin(GL_POINTS);		// 点を打っていく
	{
		const XnDepthPixel* pDepth = _pDepth;		// デプスデータのポインタ取得
		const XnRGB24Pixel* pImage = _pImage;
		const XnLabel* pLabel = _pLabel;			// 人物データのポインタ取得
		XnPoint3D* point = _pPoint;

		for(XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; y ++){
			for(XnUInt x = 0; x < KINECT_IMAGE_WIDTH; x ++, pDepth ++, pImage ++, pLabel ++, point ++){
				// ポイントの設定
				point->X = x;
				point->Y = y;

				if(*pDepth == 0){		// 深さが0なら一番奥に配置
					point->Z = KINECT_MAX_DEPTH - HUMAN_FOWARD_NUM;
				}else{
#ifdef FOWARD_FLAG
					point->Z = 50;
#else 
					point->Z = *pDepth;
#endif
				}

				// 描画
				if(*pLabel != 0){
					glColor3ubv((GLubyte*)pImage);		// 色の表示
					glVertex3fv((GLfloat*)point);		// 点の表示
				}
			}
		}
	}
	glEnd();	// 点を打つのを終了する
}
