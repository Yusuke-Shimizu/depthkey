/******************************************

メモ

ヘッダ（クラス）分けをしたい

グローバル変数減らす

ユーザクラス作る

使える背景
13(A : 1.0, B : 2.0)
15(A : 1.0, B : 3.0)
16(A : 2.0, B : 2.0)

*******************************************/


//---------------------------------------------------------------------------
// インクルード
//---------------------------------------------------------------------------
#include <XnOS.h>
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <glut.h>	// OpenGL
#endif

#include <iostream>		// 入出力
#include <sstream>		// 文字列ストリーム
#include <fstream>
#include <stdexcept>	// 例外.
#include <direct.h>		// ディレクトリヘッダ

#include <math.h>			// 数学系ヘッダ
#include <XnCppWrapper.h>	// OpenNI
#include <xnFPSCalculator.h>// FPS測定用

#include "gl_screenshot.h"	// スクリーンショットヘッダ
#include "BMPLoader.h"		// bmp読み込み用
#include "draw.h"

//---------------------------------------------------------------------------
// 名前空間
//---------------------------------------------------------------------------
using namespace xn;
using std::cout;
using std::endl;

//---------------------------------------------------------------------------
// 定義
//---------------------------------------------------------------------------
//#define INIT_PATH "../Kinect/"
#define SAMPLE_XML_PATH "../Kinect/Data/SamplesConfig.xml"
#define BACK_IMAGE_PATH "../Kinect/Data/background/chromaBack16.bmp"
#define BACK_DEPTH_PATH "../Kinect/Data/background/chromaBack16.txt"
#define PLAYER_RECORDE_PATH "../Kinect/Data/player.oni"

#define OUT_IMAGE_PATH "../Kinect/bitmap/chromaBack17.bmp"
#define OUT_DEPTH_PATH "../Kinect/bitmap/chromaBack17.txt"
#define OUT_RECORDE_PATH "../Kinect/Data/record.oni"

#define GL_WIN_SIZE_X 1280	// ウィンドウズの幅
#define GL_WIN_SIZE_Y 1024	// ウィンドウズの高さ

#define KINECT_IMAGE_WIDTH	640		// 画像の幅(image(depth)MD.XRes)
#define KINECT_IMAGE_HEIGHT	480		// 画像の高さ(image(depth)MD.YRes)
#define KINECT_IMAGE_SIZE	307200	// 画像の大きさ(640 * 480)
#define KINECT_MAX_DEPTH 10000		// デプスの最大値
#define KINECT_VISIBLE_DELTA 10		// 可視用差異

#define DISPLAY_MODE_OVERLAY		1				// イメージとデプスのオーバレイモード
#define DISPLAY_MODE_DEPTH			2				// デプスモード
#define DISPLAY_MODE_IMAGE			3				// イメージモード
#define DISPLAY_MODE_CHROMA			4				// クロマキーモード
#define DISPLAY_MODE_POINT_CLOUD	5				// ポイントクラウドモード
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_IMAGE	// デフォルトのモード

#define SHOW_MODE_NORMAL			1					// イメージモード
#define SHOW_MODE_POINT_CLOUD		2					// クロマキーモード
#define DEFAULT_SHOW_MODE			SHOW_MODE_NORMAL	// デフォルトのモード

#define GL_CHAR_SIZE 20		// 文字サイズ
#define DEBUG_X 15			// デバッグの初期位置X
#define DEBUG_Y 25			// デバッグの初期位置Y

#define ZOOM_RATE 1			// 撮影時の深度拡大率

#define GL_GENERATE_MIPMAP_SGIS 0x8191	// 要検証

//---------------------------------------------------------------------------
// マクロ, inline
//---------------------------------------------------------------------------
/*#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}*/

// RGBピクセルの初期化
inline XnRGB24Pixel xnRGB24Pixel( int r, int g, int b ){
  XnRGB24Pixel pixel = { r, g, b };
  return pixel;
}

//---------------------------------------------------------------------------
// 警告の抑制
//---------------------------------------------------------------------------
//1>c:\users\shimizu\documents\visual studio 2008\projects\kinect\kinect\main.cpp(496) : warning C4996: 'xn::Context::InitFromXmlFile': Use other overload!
//1>        c:\program files (x86)\openni\include\xncppwrapper.h(5510) : 'xn::Context::InitFromXmlFile' の宣言を確認してください。
#pragma warning(disable:4996)

//---------------------------------------------------------------------------
// グローバル変数
//---------------------------------------------------------------------------
float g_pDepthHist[KINECT_MAX_DEPTH];
XnRGB24Pixel* g_pTexMap = NULL;						// 描画するとこの（テクスチャ）情報のポインタ
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;
unsigned int g_timeNum = 10000;						// ファイルの数字
unsigned int g_chromaThresh = 3000;					// クロマキー1のスレッショルド値
GLfloat g_pointSize = 2.500001;						// 点描画の点の大きさ
int g_currentWindowSizeX, g_currentWindowSizeY;		// 現在のウィンドウサイズ

unsigned int g_nViewState = DEFAULT_DISPLAY_MODE;	// 描画のモード
unsigned int g_nShowState = DEFAULT_SHOW_MODE;		// 表示のモード
bool g_debugMode = true;							// デバッグモード
bool g_fullScreenMode = true;						// フルスクリーンモード
bool g_screenShotImageMode = false;					// スクリーンショットイメージモード
bool g_screenShotDepthMode = false;					// スクリーンショットデプスモード

GLdouble g_lokEyeX = 0, g_lokEyeY = 0, g_lokEyeZ = -1;				// 視点元座標
GLdouble g_lokDirX = 0, g_lokDirY = 0, g_lokDirZ = g_lokEyeZ - 1;	// 視点先座標
XnPoint3D* g_pPoint = NULL;											// 座標を入れるポインタ

gl_screenshot g_glScreenShot;		// bmpファイルの出力
BMPImage g_back;					// 背景のbmpファイル
XnRGB24Pixel* g_pBackTex = NULL;	// 背景のイメージポインタ
XnPoint3D* g_pBackPoint = NULL;		// 背景画像の座標を入れるポインタ
XnDepthPixel* g_pBackDepth = NULL;	// 背景画像の深さを入れるポインタ

XnFPSData g_xnFPS;					// FPS計測用データ

Context g_context;
DepthGenerator g_depth;		// デプスジェネレータ
ImageGenerator g_image;		// イメージジェネレータ
UserGenerator g_user;		// ユーザジェネレータ
DepthMetaData g_depthMD;	// デプスの実データ
ImageMetaData g_imageMD;	// イメージの実データ
SceneMetaData g_sceneMD;	// ユーザの実データ

Recorder g_recorder;	// 録画用レコーダー
Player g_player;		// 再生用プレイヤー

// ユーザの色
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
// 関数プロトタイプ（後に呼び出す関数名と引数の宣言）
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
// コード
//---------------------------------------------------------------------------
//----------------------------------------------------
// メイン関数
//----------------------------------------------------
int main(int argc, char* argv[]){
	// 各種初期化
	xnInit();													// OpenNI関連の初期化
	glInit(&argc, argv);
	backInit();
	
	// Per frame code is in glutDisplay
	glutMainLoop();

	return 0;
}

//----------------------------------------------------
// OpenNI関連の初期化
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

	rc = xnFPSInit(&g_xnFPS, 180);	// FPSの初期化
	//CHECK_RC(rc, "FPS Init");

	// デプス・イメージ・ユーザジェネレータの作成
	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	errorCheck(rc, "g_depth");		// エラーチェック
	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	errorCheck(rc, "g_image");
	rc = g_context.FindExistingNode(XN_NODE_TYPE_USER, g_user);
	//rc = g_user.Create(g_context);
	errorCheck(rc, "g_user");

	// ユーザー検出機能をサポートしているか確認
	if (!g_user.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		//throw std::runtime_error("ユーザー検出をサポートしてません");
		cout << "ユーザー検出をサポートしてません" << endl;
		exit(1);
	}

	// レコーダーの設定
	//rc = setRecorder(g_recorder, rc);

	// ユーザコールバックの登録
	XnCallbackHandle userCallbacks;
	g_user.RegisterUserCallbacks(UserDetected, UserLost, NULL, userCallbacks);

	// デプス・イメージ・ユーザデータの取得
	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);
	g_user.GetUserPixels(0, g_sceneMD);

	// Hybrid mode isn't supported in this sample
	// イメージとデプスの大きさが違うとエラー
	if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes()){
		printf ("The device depth and image resolution must be equal!\n");
		exit(1);
	}

	// RGB is the only image format supported.
	// フォーマットの確認
	if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24){
		printf("The device image format must be RGB24\n");
		exit(1);
	}

	// Texture map init
	// フルスクリーン画面の大きさ調整
	g_nTexMapX = (((unsigned short)(g_depthMD.FullXRes() - 1) / 512) + 1) * 512;	// 大きさによって512の倍数に調整(1024)
	g_nTexMapY = (((unsigned short)(g_depthMD.FullYRes() - 1) / 512) + 1) * 512;	// 512
	g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));	// スクリーンの大きさ分の色情報の容量を確保

	// 座標ポインタの初期化
	g_pPoint = (XnPoint3D*)malloc(KINECT_IMAGE_SIZE * sizeof(XnPoint3D));			// 座標を入れるポインタを作成
	g_pBackTex = (XnRGB24Pixel*)malloc(KINECT_IMAGE_SIZE * sizeof(XnRGB24Pixel));	// 背景画像を入れるポインタを作成
	g_pBackPoint = (XnPoint3D*)malloc(KINECT_IMAGE_SIZE * sizeof(XnPoint3D));		// 背景座標を入れるポインタを作成
	g_pBackDepth = (XnDepthPixel*)malloc(KINECT_IMAGE_SIZE * sizeof(XnDepthPixel));		// 背景座標を入れるポインタを作成
}

//----------------------------------------------------
// openGLの初期化
//----------------------------------------------------
void glInit(int *argcp, char **argv){
	// OpenGL init
	glutInit(argcp, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);	//ディスプレイモードの指定
	glutInitWindowSize(KINECT_IMAGE_WIDTH, KINECT_IMAGE_HEIGHT);			//ウィンドウサイズの指定
	glutCreateWindow ("Kinect Viewer");							//ウィンドウの作成
	//glutFullScreen();											// フルスクリーンにする
	//glutSetCursor(GLUT_CURSOR_NONE);							// カーソルのイメージを消す

	glutCallback();		// コールバック
}

//----------------------------------------------------
// 背景の初期化
//----------------------------------------------------
void backInit(void){
	// 背景のイメージデータ初期化
	g_back.Load(BACK_IMAGE_PATH);		// 背景画像の初期化

	// BMPImage(g_back) -> XnRGB24Pixel(g_pBackTex)
	XnRGB24Pixel* pBack = g_pBackTex;													// 背景画像を入れるポインタ
	GLuint g_backWidth = g_back.GetWidth();												// 背景の横幅の大きさ
	GLubyte* pBackData = g_back.GetData() + g_back.GetImageSize() - 3 * g_backWidth;	// 背景のポインタ取得(最後から見ていく)

	for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){	// 480
		for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pBack){	// 640
			// 深さが0か閾値以上なら背景画像を描画（閾値以下ならその部分を残す）
			pBack->nRed		= *pBackData;
			pBack->nGreen	= *(pBackData + 1);
			pBack->nBlue	= *(pBackData + 2);

			pBackData += 3;		// RGBの3つ分進める
		}

		pBackData -= 2 * 3 * g_backWidth;	// 一行のRGB3つ分を二つ（2行）分戻る
	}

	// 背景のデプスデータ(g_pBackData)初期化
	//g_backDepthData.open();	// 背景の深さデータを開く
	ifstream backDepthData(BACK_DEPTH_PATH);			// 背景の深さデータ
	
	//XnPoint3D* pBackPoint = g_pBackPoint;
	XnDepthPixel* pBackDepth = g_pBackDepth;

	// 背景の座標値を取得
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
// プレイヤーの初期化
//----------------------------------------------------
void playerInit(void){
	XnStatus rc;

	rc = g_context.Init();
	rc = g_context.OpenFileRecording(PLAYER_RECORDE_PATH, g_player);
	errorCheck(rc, "OpenFileRecording");		// エラーチェック

	rc = g_context.FindExistingNode(XN_NODE_TYPE_PLAYER, g_player);
	errorCheck(rc, "FindExistingNode_player");

	// サポートされるフォーマットの取得
	cout << "SupportedFormat:" <<
		g_player.GetSupportedFormat() << endl;

	// 再生速度の取得
	cout << "PlaybackSpeed:" << g_player.GetPlaybackSpeed() << endl;
}

//---------------------------------------------------------------------------
// コールバック関数
//---------------------------------------------------------------------------
//----------------------------------------------------
// アイドル時の処理
//----------------------------------------------------
void glutIdle (void){
	// Display the frame
	glutPostRedisplay();
}

//----------------------------------------------------
// 描画処理
//----------------------------------------------------
void glutDisplay (void){
	xnFPSMarkFrame(&g_xnFPS);		// FPSの計測開始？

	XnStatus rc = XN_STATUS_OK;

	// 更新されたノードを待つ(どれでもいい)
	rc = g_context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK){
		printf("Read failed: %s\n", xnGetStatusString(rc));
		printf("test\n");
		return;
	}

	// イメージ・デプス・ユーザのデータを取得
	g_image.GetMetaData(g_imageMD);
	g_depth.GetMetaData(g_depthMD);
	g_user.GetUserPixels(0, g_sceneMD);

	// カラー・デプスバッファをクリア
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 設定
	setDepthHistgram(g_depth, g_depthMD, g_pDepthHist);	// ヒストグラムの計算・作成
	setTexture();	// テクスチャ設定

	// 描画
	drawImage();	// イメージデータの描画

	// デバッグモードの文字は描画の行列と隔離
	glMatrixMode(GL_PROJECTION);								// 射影変換の行列の設定
	//glLoadIdentity();											// スタックのクリア
	glMatrixMode(GL_MODELVIEW);								// モデルビュー変換の行列の設定
	glLoadIdentity();
	if(g_debugMode) glDebug();								// デバッグモード

	// 一度だけスクリーンショットをとる
	if(g_screenShotImageMode){
		ostringstream fname;
		fname	<< OUT_IMAGE_PATH ;//出力ファイル名
		std::string name = fname.str();
		g_glScreenShot.screenshot(name.c_str(), 24);

		g_screenShotImageMode = !g_screenShotImageMode;	// トグル
	}

	// 一度だけ深さデータを取得する
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

		g_screenShotDepthMode = !g_screenShotDepthMode;	// トグル
	}

	// Swap the OpenGL display buffers
	glutSwapBuffers();
}

//----------------------------------------------------
// リサイズ処理
//----------------------------------------------------
void glutResize (int w, int h){
	// 変更後のウィンドウサイズを取得
	g_currentWindowSizeX = w;
	g_currentWindowSizeY = h;

	// ウィンドウ全体をビューポートにする
	glViewport(0, 0, w, h);

	// ポイントサイズの変更
	//g_pointSize = ;
	GLfloat tmp1 = (w * w * 6.10364 / 10000000.0) + (w * 0.00117187);
	GLfloat tmp2 = (h * h * 6.5824 / 10000000.0) + (h * 0.00176738);

	// 点が大きい方を設定する
	//if(tmp1 > tmp2){
		g_pointSize = tmp1;
	//}else{
	//	g_pointSize = tmp2;
	//}

	cout << "resize(" << w << ", " << h << "), point = " << g_pointSize << endl;

	// 変換行列の初期化
	//glLoadIdentity();
}

//----------------------------------------------------
// キーボード処理
//----------------------------------------------------
void glutKeyboard (unsigned char key, int x, int y){
	switch (key){
		case '1':	// オーバーレイモード
			g_nViewState = DISPLAY_MODE_OVERLAY;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// イメージとデプスのずれを無くす
			break;
		case '2':	// デプスモード
			g_nViewState = DISPLAY_MODE_DEPTH;
			g_depth.GetAlternativeViewPointCap().ResetViewPoint();		// デプスの大きさを元に戻す？
			break;
		case '3':	// イメージモード
			g_nViewState = DISPLAY_MODE_IMAGE;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// イメージとデプスのずれを無くす
			break;
		case '4':	// クロマキーモード
			g_nViewState = DISPLAY_MODE_CHROMA;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// イメージとデプスのずれを無くす
			break;
		case '5':	// ポイントクラウドモード
			g_nViewState = DISPLAY_MODE_POINT_CLOUD;
			g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);	// イメージとデプスのずれを無くす
			break;
			
		case 'm':	// ★連打厳禁！！（何故か重くなる）★
			g_context.SetGlobalMirror(!g_context.GetGlobalMirror());	// ミラーリング
			break;
		case 'd':
			g_debugMode = !g_debugMode;									// デバッグモードのオンオフ
			break;

		case 'f':
			g_fullScreenMode = !g_fullScreenMode;	// フルスクリーンモードの切り替え（トグルスイッチ）

			if(g_fullScreenMode){					// フルスクリーンモード
				glutFullScreen();
			}else{									// ウィンドウモード
				glutPositionWindow(100, 100);
				glutReshapeWindow(KINECT_IMAGE_WIDTH, KINECT_IMAGE_HEIGHT);
			}
			break;

		case 's':	// スクリーンショットを撮る（深さも撮る）
			g_screenShotDepthMode = true;
		case 'S':	// スクリーンショットを撮る（深さは撮らない）
			g_screenShotImageMode = true;
			break;

		case 'R':	// レコードストップ
			g_recorder.RemoveNodeFromRecording(g_image);
			g_recorder.RemoveNodeFromRecording(g_depth);

			cout << "recording stop!" << endl;
			break;

		// 閾値の増減
		case 't': g_chromaThresh += 10; break;
		case 'T': g_chromaThresh -= 10; break;

		//case 'p':
		//	g_pointSize += 0.000001;
		//	break;
		//case 'P':
		//	g_pointSize -= 0.000001;
		//	break;

		// 視点移動
		case 'x': g_lokEyeX ++; g_lokDirX ++; break;
		case 'X': g_lokEyeX --; g_lokDirX --; break;
		case 'y': g_lokEyeY ++; g_lokDirY ++; break;
		case 'Y': g_lokEyeY --; g_lokDirY --; break;
		case 'z': g_lokEyeZ += 1; g_lokDirZ += 1; break;
		case 'Z': g_lokEyeZ -= 1; g_lokDirZ -= 1; break;

		// 強制終了
		case 27:	// Escボタン
		case 'q':
		case 'Q':
			exit (1);
	}
}

//----------------------------------------------------
// マウスのクリック処理
//----------------------------------------------------
void glutMouse(int button, int state, int _x, int _y){
	int x = _x, y = _y;
	XnPoint3D pt[2] = {{0,0,0},{0,0,0}};

	// サイズが違う場合，680*480に標準化する
	if(!(g_currentWindowSizeX == KINECT_IMAGE_WIDTH && g_currentWindowSizeY == KINECT_IMAGE_HEIGHT)){
		x = 640 * _x / g_currentWindowSizeX;
		y = 480 * _y / g_currentWindowSizeY;
	}

	if(state == GLUT_DOWN){
		if(button == GLUT_LEFT_BUTTON){			// 左クリック
			cout << "click! (" << _x << ", " << _y << ")->(" << x << ", " << y << "), depth = " << *(g_depthMD.Data() + y * KINECT_IMAGE_WIDTH + x) << endl;
			pt[0].X = _x;
			pt[0].Y = _y;
			pt[0].Z = *(g_depthMD.Data() + y * KINECT_IMAGE_WIDTH + x);
			//cout << "(_x, _y) -> (x, y) = (" << _x << ", " << _y << ") -> (" << x << ", " << y << ")" << endl;

			g_depth.ConvertProjectiveToRealWorld(2, pt, pt);

			cout << "change pt[0] => (" << pt[0].X << ", " << pt[0].Y << ", " << pt[0].Z << ")" << endl;
		}else if(button == GLUT_RIGHT_BUTTON){	// 右クリック
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
// コールバック関数のまとめ
//----------------------------------------------------
void glutCallback(void){
	glutKeyboardFunc(glutKeyboard);		// キーボード入力時に呼び出される関数を指定する（関数名：glutKeyboard）
	glutMouseFunc(glutMouse);			// 
	glutDisplayFunc(glutDisplay);		// 描画時に呼び出される関数を指定する（関数名：glutDisplay）
	glutReshapeFunc(glutResize);		// ウィンドウサイズ変更時に呼び出される関数の指定
	glutIdleFunc(glutIdle);				// プログラムアイドル状態時に呼び出される関数
}

//----------------------------------------------------
// レコーダーの設定
//----------------------------------------------------
XnStatus setRecorder(Recorder recorder, XnStatus rc){
	//XnStatus rc;

	// レコーダーの作成
	rc = recorder.Create(g_context);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}

	// 記録設定
	rc = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, OUT_RECORDE_PATH);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}
    
	// イメージを記録対象に追加
	rc = recorder.AddNodeToRecording(g_image, XN_CODEC_JPEG);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}
    
	// デプスを記録対象に追加
	rc = recorder.AddNodeToRecording(g_depth, XN_CODEC_UNCOMPRESSED);
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		std::cout << __LINE__ << std::endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}
    
	// 記録開始(WaitOneUpdateAllのタイミングで記録される)
	rc = recorder.Record();
	if (rc != XN_STATUS_OK) {
		cout << "error!" << endl;
		throw std::runtime_error(xnGetStatusString(rc));
	}

	cout << "record set ok!" << endl;

	return rc;
}

//----------------------------------------------------
// デバッグ関数
//----------------------------------------------------
void glDebug(void){
	int deb_x = DEBUG_X, deb_y = DEBUG_Y;
	ostringstream oss;

	glColor3f(0, 0, 1);

	glPrintString("DEBUG MODE", deb_x, deb_y);

	oss << "VIEW MODE -> " << g_nViewState;									// viewモードの表示
	glPrintString(oss.str().c_str(), deb_x += GL_CHAR_SIZE, deb_y += GL_CHAR_SIZE * 2);
	oss.str("");	// バッファのクリア
	oss << "windowSize -> (" << g_currentWindowSizeX << ", " << g_currentWindowSizeY << ")";	// 現在のウィンドウサイズの表示
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE);
	oss.str("");

	oss << "FPS : " << xnFPSCalc(&g_xnFPS);									// FPSの表示
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE * 2);
	oss.str("");
	oss << "threshold -> " << g_chromaThresh;								// スレッショルドの表示
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE);
	oss.str("");
	oss << "Look (" << g_lokEyeX << "," << g_lokEyeY << "," << g_lokEyeZ << ") -> (" 
		<< g_lokDirX << "," << g_lokDirY << "," << g_lokDirZ << ")";		// 視点の表示
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE);	
	oss.str("");

	oss << "g_pointSize = " << g_pointSize;									// 点の大きさの表示
	glPrintString(oss.str().c_str(), deb_x, deb_y += GL_CHAR_SIZE * 2);
	oss.str("");
}

//----------------------------------------------------
// ヒストグラム作成関数
//----------------------------------------------------
void setDepthHistgram(const xn::DepthGenerator& depth, const xn::DepthMetaData& depthMD, float _pDepthHist[]){
	xnOSMemSet(_pDepthHist, 0, KINECT_MAX_DEPTH * sizeof(float));	// g_pDepthHistの全てに0を代入

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
// テクスチャの設定
//----------------------------------------------------
void setTexture(void){
	xnOSMemSet(g_pTexMap, 0, g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));	// g_pTexMapの全てに0を代入

	// 描画モード1か3
	if (g_nViewState == DISPLAY_MODE_OVERLAY || g_nViewState == DISPLAY_MODE_IMAGE){
		const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();	// g_imageMDのポインタ取得(画像データ取得)
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

	// 描画モード1か2
	if (g_nViewState == DISPLAY_MODE_OVERLAY || g_nViewState == DISPLAY_MODE_DEPTH){
		const XnDepthPixel* pDepthRow = g_depthMD.Data();
		XnRGB24Pixel* pTexRow = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;
		const XnLabel* pLabel = g_sceneMD.Data();

		for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){
			const XnDepthPixel* pDepth = pDepthRow;
			XnRGB24Pixel* pTex = pTexRow + g_depthMD.XOffset();

			for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pDepth, ++ pTex, ++ pLabel){
				int nHistValue = g_pDepthHist[*pDepth];

				if(*pLabel){		// 人物なら
					*pTex = userColor[*pLabel];
				}else if (*pDepth != 0){
					if(*pDepth < 1000){
						*pTex = xnRGB24Pixel(nHistValue, 0, 0);		// red
					}else if(*pDepth < 2000){
						*pTex = xnRGB24Pixel(0, nHistValue, 0);		// green
					}else if(*pDepth < 3000){
						*pTex = xnRGB24Pixel(0, 0, nHistValue);		// blue
					}else if(*pDepth < 4000){
						*pTex = xnRGB24Pixel(nHistValue, nHistValue, 0);	// 水色
					}else if(*pDepth < 5000){
						*pTex = xnRGB24Pixel(0, nHistValue, nHistValue);	// yellow
					}else{
						*pTex = xnRGB24Pixel(nHistValue, 0, nHistValue);	// 紫
					}
				}
			}

			pDepthRow += g_depthMD.XRes();
			pTexRow += g_nTexMapX;
		}
	}

	// 描画モード4
	//if (g_nViewState == DISPLAY_MODE_CHROMA){
	//	// イメージデータ(カメラ映像)貼り付け
	//	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();	// g_imageMDのポインタ取得(画像データ取得)
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

	//	// デプスデータを用いた人物抜き出し + 背景合成
	//	const XnDepthPixel* pDepthRow = g_depthMD.Data();		// デプスデータのポインタ取得
	//	pTexRow = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;
	//	GLuint g_backWidth = g_back.GetWidth();						// 背景の横幅の大きさ
	//	GLubyte* pBackData = g_back.GetData() + g_back.GetImageSize() - 3 * g_backWidth;	// 背景のポインタ取得(最後から見ていく)

	//	for (XnUInt y = 0; y < KINECT_IMAGE_HEIGHT; ++ y){	// 480
	//		const XnDepthPixel* pDepth = pDepthRow;			// デプスデータのポインタ取得
	//		XnRGB24Pixel* pTex = pTexRow + g_depthMD.XOffset();

	//		for (XnUInt x = 0; x < KINECT_IMAGE_WIDTH; ++ x, ++ pDepth, ++ pTex){	// 640
	//			// 深さが0か閾値以上なら背景画像を描画（閾値以下ならその部分を残す）
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
// イメージ描画
//----------------------------------------------------
void drawImage(void){
	switch(g_nViewState){
		case DISPLAY_MODE_OVERLAY:		// ノーマル描画モード
		case DISPLAY_MODE_DEPTH:
		case DISPLAY_MODE_IMAGE:

			glMatrixMode(GL_PROJECTION);								// 射影変換の行列の設定
			glLoadIdentity();											// スタックのクリア
			gluOrtho2D(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0);	// ワールド座標系を正規化デバイス座標系に平行投影(left, right, buttom, top, near, far)
															// ★平行投影する事によって，ポイントクラウドも平面に投影でき，クロマキーに最適
															// Kinectの距離は約500～9000まで使える(設定は10000)
			glMatrixMode(GL_MODELVIEW);						// モデルビュー変換の行列の設定
			glLoadIdentity();

			glEnable(GL_TEXTURE_2D);	// テクスチャマッピングの有効化

			// テクスチャパラメータの設定と定義
			glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, g_pTexMap);	// イメージデータ貼り付け

			// Display the OpenGL texture map
			glColor4f(1,1,1,1);

			// イメージデータの貼り付け
			glBegin(GL_QUADS);		// 四角形の描画を行う
			{
				int nXRes = g_depthMD.FullXRes();
				int nYRes = g_depthMD.FullYRes();

				// 左上
				glTexCoord2f(0, 0);
				glVertex2f(0, 0);	// 座標指定
				// 右上
				glTexCoord2f((float)nXRes/(float)g_nTexMapX, 0);
				glVertex2f(GL_WIN_SIZE_X, 0);	// 座標指定
				// 右下
				glTexCoord2f((float)nXRes/(float)g_nTexMapX, (float)nYRes/(float)g_nTexMapY);
				glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);	// 座標指定
				// 左下
				glTexCoord2f(0, (float)nYRes/(float)g_nTexMapY);
				glVertex2f(0, GL_WIN_SIZE_Y);	// 座標指定
			}
			glEnd();

			glDisable(GL_TEXTURE_2D);	// テクスチャマッピングの無効化

			break;

		case DISPLAY_MODE_CHROMA:		// ポイントクラウド描画モード
		case DISPLAY_MODE_POINT_CLOUD:

			// 投影変換
			glMatrixMode(GL_PROJECTION);								// 射影変換の行列の設定
			glLoadIdentity();											// スタックのクリア
			glOrtho(0, KINECT_IMAGE_WIDTH, 
				KINECT_IMAGE_HEIGHT, 0, 
				-1.0, -KINECT_MAX_DEPTH - KINECT_VISIBLE_DELTA);	// ワールド座標系を正規化デバイス座標系に平行投影(left, right, buttom, top, near, far)
																	// ★平行投影する事によって，ポイントクラウドも平面に投影でき，クロマキーに最適
																	// Kinectの距離は約500～9000まで使える(設定は10000)
			// 視野変換
			gluLookAt(
				g_lokEyeX, g_lokEyeY, g_lokEyeZ,	// 視点の位置(初期位置：(0,0,-1))
				g_lokDirX, g_lokDirY, g_lokDirZ,	// 視点先の位置(初期位置：(0,0,-2))
				0.0, 1.0, 0.0);						// 向き
	 
			// モデリング変換
			glMatrixMode(GL_MODELVIEW);								// モデルビュー変換の行列の設定
			glLoadIdentity();										// スタックのクリア

			glEnable(GL_DEPTH_TEST);	// 陰面処理の有効化

			// ポイントクラウド表示
			glPointSize(g_pointSize);			// 点のサイズ
			drawPointCloud(g_pBackTex, g_pBackDepth, g_pPoint);					//背景画像表示
			//drawPointCloud(g_imageMD.RGB24Data(), g_depthMD.Data(), 10, g_chromaThresh);	// 人物抜き出し(距離の閾値)
			drawPointCloudHuman(g_imageMD.RGB24Data(), g_depthMD.Data(), g_sceneMD.Data(), g_pPoint);	// 人物抜き出し(動くものを検出)

			glDisable(GL_DEPTH_TEST);	// 陰面処理の無効化
			break;

	}
}

//----------------------------------------------------
// ユーザー検出
//----------------------------------------------------
void XN_CALLBACK_TYPE UserDetected(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	cout << "ユーザー検出:" << nId << " " << generator.GetNumberOfUsers() << "人目" << endl;
}

//----------------------------------------------------
// ユーザー消失
//----------------------------------------------------
void XN_CALLBACK_TYPE UserLost(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	cout << "ユーザー消失:" << nId << endl;
}

//----------------------------------------------------
// エラーチェック
//----------------------------------------------------
void errorCheck(XnStatus status, char* what){
	if (status != XN_STATUS_OK) {
		cout << "error! : " << what << endl;
		//throw std::runtime_error(xnGetStatusString(status));
		exit(1);
	}	
}
