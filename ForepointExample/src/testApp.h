#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxControlPanel.h"

#include "DatumHistorian.h"
#include "gmlStroke.h"


// include these 2 lines for median-of-5 code:
#include <algorithm>
using std::swap;

	


class testApp : public ofBaseApp
{
	

	public:

		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	


	ofxControlPanel			gui;
	ofxKinect				kinect;
	
	
	//--------------------------------------------
	vector<gmlStroke>		strokes;
	int						nStrokes;
	bool					bRecordingAStroke;
	void					startStroke();
	void					endStroke();
	void					updateStroke(float x, float y, float z);
	void					updateStroke2 (ofPoint pt);
	void					drawStrokes (float ox, float oy, float ow, float oh);
	void					clearStrokes();
	void					saveStrokesGML();
	
	//--------------------------------------------
	int						KW;
	int						KH;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthBlobs;
	ofxCvGrayscaleImage		ofxCv8uC1_Depth;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthRaw;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthRawThreshed;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev1;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev2;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev3;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev4;
	ofxCvGrayscaleImage		ofxCv8uC1_Temp1;
	ofxCvGrayscaleImage		ofxCv8uC1_Temp2;
	ofxCvGrayscaleImage		ofxCv8uC1_ThreshN;
	ofxCvGrayscaleImage		ofxCv8uC1_ThreshF;
	ofxCvGrayscaleImage		ofxCv8uC1_ThreshNF;
	
	ofxCvGrayscaleImage		ofxCv8uC1_DepthBg;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthBgDif;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthBgDifThresh;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthPrev;
	ofxCvGrayscaleImage		ofxCv8uC1_DepthFrameDif;
	
	//--------------------------------------------
	ofxCvContourFinder		contourFinder;
	int						MAX_N_CONTOUR_POINTS;
	CvPoint					**cvpts;
	int						*ncvpts;
	IplImage*				cvImgTemp;
	
	//--------------------------------------------
	ofPoint					forePoint;
	ofPoint					forePointPrev;
	ofPoint					forePointSmooth;
	void					drawForePoint(float ox, float oy, float ow, float oh);
	int						nFramesOfJumpTooBig;
	DatumHistorian			*DHX;
	DatumHistorian			*DHY;
	DatumHistorian			*DHZ;
	
	//--------------------------------------------
	bool				bComputeDepthHistogram;
	int					depthHistogramSize;
	int					depthHistogramWidth;
	int					depthHistogramHeight;
	CvHistogram*		depthHistogram;
	float*				depthHistogramData;
	void				drawDepthHistogram();
	void				computeDepthHistogram (ofxCvGrayscaleImage depthImage);
	
	//--------------------------------------------
	void				fillHolesPre();
	void				fillHolesPost();
	void				captureBackground();
	void				processBackground();
	void				computeForegroundBlobs();
	void				computeForePoint(ofxCvGrayscaleImage depthImage);
	bool				bLastCaptureBg;

	//--------------------------------------------	
	int					gW;
	int					gH;
	int					gM;
	
	void				computeFrameRate();
	float				kinectPrevFrameMillis;
	float				kinectFrameRate;
	
	void				drawPointCloud();

};

#endif
