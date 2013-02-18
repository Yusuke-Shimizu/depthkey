#define KINECT_MAX_DEPTH 10000		// デプスの最大値

//---------------------------------------------------------------------------
// グローバル変数
//---------------------------------------------------------------------------

//----------------------------------------------------
// 関数プロトタイプ（後に呼び出す関数名と引数の宣言）
//----------------------------------------------------
void glPrintString(const char *str, float x = 0, float y = 0, float z = 0, void *font = GLUT_BITMAP_TIMES_ROMAN_24);
//void glPrintStringOss(ostringstream oss, float x = 0, float y = 0, float z = 0, void *font = GLUT_BITMAP_TIMES_ROMAN_24);
void drawPointCloud(const XnRGB24Pixel* _pImage, const XnDepthPixel* _pDepth, XnPoint3D* _pPoint, const GLubyte _forwardNum, const int thresh = KINECT_MAX_DEPTH);
void drawPointCloudHuman(const XnRGB24Pixel* _pImage, const XnDepthPixel* _pDepth, const XnLabel* _pLabel, XnPoint3D* _pPoint, const GLubyte _forwardNum);
