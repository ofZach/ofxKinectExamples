#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup()
{
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	kinect.enableDepthNearValueWhite(true);
	
	DHX = new DatumHistorian(16);
	DHY = new DatumHistorian(16);
	DHZ = new DatumHistorian(16);
	
	KW = kinect.width;
	KH = kinect.height;

	ofxCv8uC1_Temp1.allocate		(KW,KH);
	ofxCv8uC1_Temp2.allocate		(KW,KH);
	
	ofxCv8uC1_Depth.allocate		(KW,KH);
	ofxCv8uC1_DepthRaw.allocate		(KW,KH);
	ofxCv8uC1_DepthRawThreshed.allocate	(KW,KH);
	ofxCv8uC1_DepthPrev1.allocate	(KW,KH);
	ofxCv8uC1_DepthPrev2.allocate	(KW,KH);
	ofxCv8uC1_DepthPrev3.allocate	(KW,KH);
	ofxCv8uC1_DepthPrev4.allocate	(KW,KH);
	
	ofxCv8uC1_DepthBlobs.allocate	(KW,KH);
	ofxCv8uC1_ThreshN.allocate		(KW,KH);
	ofxCv8uC1_ThreshF.allocate		(KW,KH);
	ofxCv8uC1_ThreshNF.allocate		(KW,KH);
	
	ofxCv8uC1_DepthBg.allocate		(KW,KH);
	ofxCv8uC1_DepthBgDif.allocate	(KW,KH);
	ofxCv8uC1_DepthBgDifThresh.allocate	(KW,KH);
	
	ofxCv8uC1_DepthPrev.allocate	(KW,KH);
	ofxCv8uC1_DepthFrameDif.allocate(KW,KH);
	
	
	cvImgTemp = cvCreateImage( cvSize(KW,KH), IPL_DEPTH_8U, 1); 
	MAX_N_CONTOUR_POINTS = 4000;
	cvpts    = new CvPoint*[1];
	cvpts[0] = new CvPoint[MAX_N_CONTOUR_POINTS];
	for (int i=0; i<MAX_N_CONTOUR_POINTS; i++){
		cvpts[0][i] = cvPoint(0,0);
	}
	ncvpts = new int[1];
	ncvpts[0] = 0;
	
	
	//-------------------------------
	nStrokes = 0;
	bRecordingAStroke = false;
	strokes.clear();
	

	//-------------------------------
	depthHistogramSize = 256;
	depthHistogramWidth = depthHistogramSize;
	depthHistogramHeight = 96;
	float range_0[]={0, depthHistogramSize};
	float* histRanges[] = { range_0 };	
	depthHistogram = cvCreateHist(1, &depthHistogramSize, CV_HIST_ARRAY, histRanges, 1); 
	depthHistogramData = new float[depthHistogramSize];
	
	kinectPrevFrameMillis = 0;
	nFramesOfJumpTooBig = 0;
	
	gW = 256;
	gH = (gW*3)/4;
	gM = 8; 

	
	// ofSetFrameRate(60);
	ofSetVerticalSync(false);
	bComputeDepthHistogram	= false;
	
	
	
		
	gui.setup("App Controls", gM, gH*2+gM*3, gW, 300);
	gui.addPanel(" Main Controls", 1, false);
	gui.addPanel(" Hole Filling", 1, false);
	gui.addPanel(" Foreground Finding", 1, false);
	gui.addPanel(" ForePoint Smoothing", 1, false);
	
	//--------- PANEL 1
	gui.setWhichPanel(0);
	gui.setWhichColumn(0);
	gui.addSlider("Far Threshold",		"LO_THRESHOLD", 30, 0, 255, true);	
	gui.addSlider("Near Threshold",		"HI_THRESHOLD", 255, 0, 255, true);	
	gui.addToggle("DoDepth BgSubtract?","DO_DEPTH_BGSUB", 1);
	gui.addToggle("Capture DepthBg (x)","CAPTURE_DEPTH_BG", 0);
	gui.addToggle("Fill Holes?",		"FILL_HOLES", 0);
	gui.addToggle("Compute Histogram?", "DO_HISTOGRAM", 0);
	gui.loadSettings("controlPanelSettings.xml");
	
	//--------- PANEL 2
	gui.setWhichPanel(1);
	gui.setWhichColumn(0);
	gui.addSlider("# Dilations",		"N_DILATIONS", 1, 0, 5, true);	
	gui.addSlider("Depth History",		"N_HISTORY",   2, 0, 4, true);	
	gui.addToggle("Use Median?",		"DO_DEPTH_MEDIAN", 0);
	
	//--------- PANEL 3
	gui.setWhichPanel(2);
	gui.setWhichColumn(0);
	gui.addSlider("BgSub Threshold",	"BG_SUB_THRESHOLD", 3, 0,25, true);
	gui.addSlider("Min ForeGd Blob Area",	"FG_MIN_SIZE",		30, 1, 100, true);	
	
	//--------- PANEL 4
	gui.setWhichPanel(3);
	gui.setWhichColumn(0);
	gui.addSlider("ForePointAlpha",		"FORE_POINT_ALPHA",  0.600, 0.0, 0.99, false);
	gui.addSlider("ForePointMedian",	"FORE_POINT_MEDIAN", 5,1, 7, true);
	gui.addSlider("MaxJumpSize",		"MAX_JUMP_SIZE",	 50, 1, 300, true);
	gui.addToggle("PreventBigJumps",	"DO_PREVENT_JUMPS", 1); 
	

}




//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(100, 100, 100);
	gui.update();
	kinect.update();
	if (kinect.isFrameNew()){
		
		// Compute Kinect frame rate
		computeFrameRate();
		
		// Fetch values from the control panel
		int nearThreshold		= gui.getValueI("LO_THRESHOLD");
		int farThreshold		= gui.getValueI("HI_THRESHOLD");
		bool bFillHoles			= gui.getValueB("FILL_HOLES", 0);	
		bComputeDepthHistogram	= gui.getValueB("DO_HISTOGRAM", 0);
	
		bool bDoBgSubtraction   = gui.getValueB("DO_DEPTH_BGSUB");
		bool bCurrCaptureBg		= gui.getValueB("CAPTURE_DEPTH_BG");
		bool bDoCaptureTheBg	= ((bCurrCaptureBg != bLastCaptureBg) && bCurrCaptureBg);
		bLastCaptureBg = bCurrCaptureBg;
		
		// Swap from the current depth into the previous depth buffer.
		ofxCv8uC1_DepthPrev = ofxCv8uC1_Depth;
		if (bFillHoles){ 
			fillHolesPre();  // see holeFilling.cpp
		}
		
		// Retrieve the current depth buffer.
		ofxCv8uC1_DepthRaw.setFromPixels(kinect.getDepthPixels(), KW,KH);
		ofxCv8uC1_DepthRaw.mirror(false, true);

		// Compute a double-ended threshold of the depth image
		ofxCv8uC1_ThreshN = ofxCv8uC1_DepthRaw;
		ofxCv8uC1_ThreshF = ofxCv8uC1_DepthRaw;
		ofxCv8uC1_ThreshN.threshold(nearThreshold, false);
		ofxCv8uC1_ThreshF.threshold(farThreshold,  true);
		cvAnd(	ofxCv8uC1_ThreshN.getCvImage(), 
				ofxCv8uC1_ThreshF.getCvImage(), 
				ofxCv8uC1_ThreshNF.getCvImage(), NULL);
		cvAnd(	ofxCv8uC1_ThreshNF.getCvImage(), 
				ofxCv8uC1_DepthRaw.getCvImage(), 
				ofxCv8uC1_DepthRawThreshed.getCvImage(), NULL);

		// Fill holes if desired.
		if (bFillHoles){
			fillHolesPost(); // see holeFilling.cpp
		} else {
			ofxCv8uC1_Depth = ofxCv8uC1_DepthRawThreshed;
		}
		ofxCv8uC1_Depth.flagImageChanged();
		
		
		//-------------------------------------------------
		if (bDoBgSubtraction){
			// Do background subtraction on the depth image. 
			// Things will run slower, but your hand doesn't have to be frontmost to be tracked.
			
			if (bDoCaptureTheBg){			
				captureBackground(); 
			}
			processBackground();
			computeForegroundBlobs();
			computeForePoint (ofxCv8uC1_DepthBlobs);
			
			if (bComputeDepthHistogram){	
				computeDepthHistogram (ofxCv8uC1_DepthBlobs);
			}
			
		} else {
			// Don't do background subtraction on the depth image. 
			// Everything will run a lot faster, but your hand really has to be frontmost. 
			gui.setValueB("CAPTURE_DEPTH_BG", false, 0);
			computeForePoint (ofxCv8uC1_Depth);
			if (bComputeDepthHistogram){
				computeDepthHistogram (ofxCv8uC1_Depth); 
			}
		}
		
		if (bRecordingAStroke){
			float x = forePointSmooth.x / KW;
			float y = forePointSmooth.y / KH;
			float z = forePointSmooth.z / 255.0;
			updateStroke(x,y,z);
		}
		

	}
}



//--------------------------------------------------------------
void testApp::drawStrokes (float ox, float oy, float ow, float oh){
	glPushMatrix();
	
	glTranslatef(ox,oy,0.01);
	glScalef(ow,oh,1.0);
	
	// all numbers in 0...1 
	glColor3f(1,1,1);
	nStrokes = strokes.size();
	for( int i=0; i<nStrokes; i++ ) {
		strokes[i].draw();
	}
	
	glPopMatrix();
	
	
}



//--------------------------------------------------------------
void testApp::startStroke(){
	strokes.push_back( gmlStroke() );
	bRecordingAStroke = true;
}
void testApp::endStroke(){
	bRecordingAStroke = false;
}
void testApp::clearStrokes(){
	bRecordingAStroke = false;
	strokes.clear();
}

//--------------------------------------------------------------
void testApp::updateStroke (float x, float y, float z){
	if (bRecordingAStroke){
		nStrokes = strokes.size();
		if (nStrokes > 0){
			strokes[nStrokes-1].pts.push_back( ofPoint(x,y,z) );
			
			int nPts = strokes[nStrokes-1].pts.size();
			
		}
	}
}






//--------------------------------------------------------------
void testApp::captureBackground(){
	bool bDoBgSubtraction = gui.getValueB("DO_DEPTH_BGSUB");
	if (bDoBgSubtraction){
		gui.setValueB("CAPTURE_DEPTH_BG", false, 0);
		ofxCv8uC1_DepthBg = ofxCv8uC1_Depth;
		for (int i=0; i<3; i++){
			ofxCv8uC1_DepthBg.dilate_3x3();
		}
	}
}

//--------------------------------------------------------------
void testApp::processBackground(){
	
	// Background subtraction: compute the difference 
	// between the current background and the stored background, 
	// and put this difference into ofxCv8uC1_DepthBgDif
	cvSub(ofxCv8uC1_Depth.getCvImage(), 
		  ofxCv8uC1_DepthBg.getCvImage(), 
		  ofxCv8uC1_DepthBgDif.getCvImage(), NULL); 
	ofxCv8uC1_DepthBgDifThresh = ofxCv8uC1_DepthBgDif;
	
	// Threshold the difference image, so that white indicates
	// places where there is a significant difference from the background. 
	// Erode this to eliminate twinkly noise, which is characteristic of 
	// the Kinect's depth images at the edges of objects. 
	int bgSubThreshold = gui.getValueI("BG_SUB_THRESHOLD", 0);
	ofxCv8uC1_DepthBgDifThresh.threshold(bgSubThreshold,  false);
	ofxCv8uC1_DepthBgDifThresh.erode_3x3();

}

//--------------------------------------------------------------
void testApp::computeForegroundBlobs(){
	// compute the contours (blobs) of the difference-from-background image. 
	// filter these by size to extract only non-noise blobs (e.g. hands).
	
	// Find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
	// Note: "find holes" is set to false so we just get the exterior contour.
	int minFgSize = gui.getValueI("FG_MIN_SIZE", 0); 
	int minContourArea = minFgSize * minFgSize;
	int maxContourArea = KW*KH;
	contourFinder.findContours(ofxCv8uC1_DepthBgDifThresh, minContourArea, maxContourArea, 10, false);
	
	
	// Into the ofxCv8uC1_DepthBlobs image, render filled versions of the blobs, as a mask
	vector<ofxCvBlob> blobs = contourFinder.blobs;
	CvScalar color_white = CV_RGB(255,255,255);
	CvScalar color_black = CV_RGB(0,0,0);
	int nBlobs = blobs.size();
	int nCvImgTempPix = cvImgTemp->widthStep * cvImgTemp->height;
	for (int i=0; i<nCvImgTempPix; i++){ cvImgTemp->imageData[i] = 0; } // blank cvImgTemp

	// For each blob, fill it with a solid color. This eliminates holes, too. 
	for (int i=0; i<nBlobs; i++){
		ofxCvBlob blobi = blobs[i];
		vector <ofPoint>    pts  = blobi.pts;    // the contour of the blob
		int                 nPts = blobi.nPts;   // number of pts;
		if (nPts > 1){
			nPts = MIN(MAX_N_CONTOUR_POINTS, nPts); 
			for (int j=0; j<nPts; j++){ 
				ofPoint pt = pts[j];
				int px = (int) pt.x; // 0...imgw
				int py = (int) pt.y; // 0...imgh
				cvpts[0][j].x = px;
				cvpts[0][j].y = py;
			}
			ncvpts[0] = nPts;
			cvFillPoly(cvImgTemp, cvpts, ncvpts, 1, color_white, 8, 0 );
		}
		
	}
	ofxCv8uC1_DepthBlobs = cvImgTemp;
	
	// Re-color this difference-from-background image such that
	// white areas are now colored with their true depth color.
	// Voila: a depth-image containing strictly "new" objects.
	cvAnd(ofxCv8uC1_DepthBlobs.getCvImage(), 
		  ofxCv8uC1_Depth.getCvImage(), 
		  ofxCv8uC1_DepthBlobs.getCvImage(), NULL);
	ofxCv8uC1_DepthBlobs.flagImageChanged();	
}


//--------------------------------------------------------------
void testApp::computeForePoint (ofxCvGrayscaleImage depthImage){
	
	// ofxCv8uC1_DepthBlobs contains just the blobs of interest. 
	// compute the maximum value in this depth image; simultaneously,
	// compute the centroid of all pixels with that brightness.
	
	int   maxVal     = -1;
	float maxValCx    = 0;
	float maxValCy    = 0; 
	float maxValCz	  = 0; 
	int maxValCount   = 0;
	int nPixels = KW*KH;
	unsigned char val;
	
	// Compute the brightest value, and the average location of that value. 
	unsigned char* depthPix = depthImage.getPixels();
	int index = 0; 
	for (int y=0; y<KH; y++){
		for (int x=0; x<KW; x++){
			val = depthPix[index++];
			
			if (val > maxVal){
				maxVal = val; 
				maxValCount = 1;
				maxValCx = x; 
				maxValCy = y;
			} else if (val == maxVal){
				maxValCount++;
				maxValCx += x;
				maxValCy += y;
			}
		}
	}
	maxValCx /= (float)maxValCount;
	maxValCy /= (float)maxValCount;
	maxValCz  = (float)maxVal;
	
	//-----------------------------------------------------------
	// Ad-hoc method of avoiding blending across big jumps
	bool bAvoidBigJumps = gui.getValueB ("DO_PREVENT_JUMPS", 0);
	int maxJumpSize     = gui.getValueI("MAX_JUMP_SIZE", 0);
	bool bJumpIsTooBig  = false;
	if (bAvoidBigJumps){
		float dx = maxValCx - forePoint.x;
		float dy = maxValCy - forePoint.y;
		float dh = sqrt(dx*dx + dy*dy); 
		bJumpIsTooBig = (dh > maxJumpSize);
	}
	if (bAvoidBigJumps && bJumpIsTooBig){
		DHX->resetHistory  (maxValCx);
		DHY->resetHistory  (maxValCy);
		DHZ->resetHistory  (maxValCz);
	} else {
		DHX->updateHistory (maxValCx);
		DHY->updateHistory (maxValCy);
		DHZ->updateHistory (maxValCz);
	}
	
	// Set forePoint with the "true" computed values.
	forePointPrev.set(forePoint);
	forePoint.set(maxValCx, maxValCy, maxValCz); 
	
	
	// Compute a smoothed version of forePoint
	float forePointAlpha  = gui.getValueF("FORE_POINT_ALPHA",  0); 
	int forePointMedian	  = gui.getValueI("FORE_POINT_MEDIAN", 0); 
	float dhx = DHX->getRunningAverageOfMedianOfN (forePointAlpha, forePointMedian);
	float dhy = DHY->getRunningAverageOfMedianOfN (forePointAlpha, forePointMedian);
	float dhz = DHZ->getRunningAverageOfMedianOfN (forePointAlpha, forePointMedian);
	forePointSmooth.set(dhx, dhy, dhz); 
}



//--------------------------------------------------------------
void testApp::computeDepthHistogram (ofxCvGrayscaleImage depthImage){
	// Compute the histogram of the depth-colored difference-from-background image. 
	
	IplImage*  iplDepthImg = depthImage.getCvImage();
	cvCalcHist( &iplDepthImg, depthHistogram, 0, NULL );
	float *depthHistArr = cvGetHistValue_1D (depthHistogram, 0);
	
	int maxVal = 0;
	int startIndex = 1; // don't count black pixels. 
	for (int i=startIndex; i<depthHistogramSize; i++){
		if (depthHistArr[i] > maxVal){
			maxVal = depthHistArr[i];
		}
	}
	
	for (int i=0; i<depthHistogramSize; i++){
		depthHistogramData[i] = depthHistArr[i] / (float)maxVal;
	}
}



//--------------------------------------------------------------
void testApp::draw()
{
	glColor3f(1,1,1);
	ofxCv8uC1_Depth.draw			(gM*1+gW*0,	gM*1,		gW, gH);
	
	if (gui.getValueB("DO_DEPTH_BGSUB")){
		ofxCv8uC1_DepthBg.draw		(gM*2+gW*1, gM*2+gH,	gW,	gH);
		ofxCv8uC1_DepthBlobs.draw	(gM*1+gW*0, gM*2+gH,	gW,	gH);
		contourFinder.draw			(gM*1+gW*0, gM*2+gH,	gW,	gH);
		
	}
	
	float sc = 2.0;//2.75;
	float gr = 1.0;
	glColor3f(gr,gr,gr);
	ofxCv8uC1_Depth.draw			(gM*3+gW*2,	gM*1,		gW*sc, gH*sc);
	//ofxCv8uC1_DepthBlobs.draw		(gM*3+gW*2,	gM*1,		gW*sc, gH*sc);
	drawStrokes						(gM*3+gW*2,	gM*1,		gW*sc, gH*sc);
	drawForePoint					(gM*3+gW*2,	gM*1,		gW*sc, gH*sc);

	
	if (bComputeDepthHistogram){ 
		drawDepthHistogram(); 
	}
	
	ofSetColor(255,128,0);
	ofDrawBitmapString("Depth",			5+gM*1+gW*0,	15+gM*1);
	ofDrawBitmapString("Background",	5+gM*2+gW*1,	15+gM*2+gH);
	ofDrawBitmapString("Foreground",	5+gM*1+gW*0,	15+gM*2+gH);
	ofDrawBitmapString("ForePoint",		5+gM*3+gW*2,	15+gM*1);
	
	ofSetColor(255,255,0);
	ofDrawBitmapString("KEYS:",			5+gM*2+gW*1,		15+gM*2+gH*2);
	ofDrawBitmapString("Start mark: a",	5+gM*2+gW*1,		30+gM*2+gH*2);
	ofDrawBitmapString("End mark: z",	5+gM*2+gW*1,		45+gM*2+gH*2);
	ofDrawBitmapString("Clear mark: space",	5+gM*2+gW*1,	60+gM*2+gH*2);
	
	
	char reportStr[1024];
	sprintf(reportStr, "fps: %f", kinectFrameRate);//ofGetFrameRate());
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(reportStr, gW*0+gM*1, ofGetHeight()-gM);
	
	gui.draw();
}


//--------------------------------------------------------------
void testApp::drawForePoint(float ox, float oy, float ow, float oh){
	
	glPushMatrix();
	glTranslatef(ox,oy, 0);
	
	float x = ofMap(forePointSmooth.x, 0,KW, 0,ow, false);
	float y = ofMap(forePointSmooth.y, 0,KH, 0,oh, false);
	
	
	ofFill();
	ofSetColor(0,200,0); 
	ofEllipse(x,y, 15,15);
	ofSetColor(200,255,0); 
	ofEllipse(x,y, 7,7);
	ofNoFill();
	
	glPopMatrix();
}





//--------------------------------------------------------------
void testApp::drawDepthHistogram(){
	
	
	glPushMatrix();
	glTranslatef(gM*2+gW*1, gM*3+gH*2,0);
	
	glColor3f(0.5,0.5,0.5);
	ofRect(0,0,depthHistogramWidth,depthHistogramHeight); 
	glColor3f(1,1,1);
	for (int i=1; i<depthHistogramSize; i++){ // skip 0 (black)
		float y = depthHistogramData[i] * depthHistogramHeight;
		ofLine(i, 0, i, y);
	}
	glPopMatrix();
}






void testApp::drawPointCloud() {
	int step = 2;
	ofScale(1. / step, 1. / step, 1. / step);
	// two magic numbers derived visually
	float scaleFactor = .0021;
	float minDistance = -10;
	int w = 640;
	int h = 480;
	ofTranslate(w / 2, h / 2);
	ofRotateY(mouseX);
	ofTranslate(-w / 2, -h / 2);
	float* distancePixels = kinect.getDistancePixels();
	glBegin(GL_POINTS);
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			int i = y * w + x;
			float z = distancePixels[i];
			float scaledx = (x - w / 2) * (z + minDistance) * scaleFactor;
			float scaledy = (y - h / 2) * (z + minDistance) * scaleFactor;
			glVertex3f(scaledx, scaledy, z);
		}
	}
	glEnd();
}

//--------------------------------------------------------------
void testApp::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key)
{
	switch (key)
	{
		
		case 'x':
		case 'X':
			captureBackground();
			break;
	
		case ' ':
			clearStrokes();
			break;
		
		case 'a':
		case 'A':
			startStroke();
			break;
		
		case 'z':
		case 'Z':
			endStroke();
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
	gui.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	//printf("%d button\n", button); 
	if (button == 2){
		// right click learns background;
		captureBackground();
	} else {
		
		gui.mousePressed(x, y, button);
		/*
		if (bRecordingAStroke){
			endStroke();
		} else {
			clearStrokes();
			startStroke();
		}
		*/
		
	}
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	gui.mouseReleased();
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
}


//--------------------------------------------------------------
void testApp::computeFrameRate(){
	float now = ofGetElapsedTimeMillis();
	float FR = 1000.0/(now - kinectPrevFrameMillis);
	float fA = 0.95; 
	float fB = 1.0-fA;
	kinectFrameRate = (fA*kinectFrameRate) + (fB*FR); 
	kinectPrevFrameMillis = now;
}










/*
 // Find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
 // Note: "find holes" is set to false so we just get the exterior contour.
 contourFinder.findContours(ofxCv8uC1_Depth, 10, KW*KH/2, 20, false);
 
 // Into the ofxCv8uC1_DepthBlobs image, render filled versions of the blobs, as a mask
 vector<ofxCvBlob> blobs = contourFinder.blobs;
 
 CvScalar color_white = CV_RGB(255,255,255);
 CvScalar color_black = CV_RGB(0,0,0);
 int nBlobs = blobs.size();
 int nCvImgTempPix = cvImgTemp->widthStep * cvImgTemp->height;
 for (int i=0; i<nCvImgTempPix; i++){ 
 cvImgTemp->imageData[i] = 0; // blank cvImgTemp
 }
 for (int i=0; i<nBlobs; i++){
 ofxCvBlob blobi = blobs[i];
 vector <ofPoint>    pts  = blobi.pts;    // the contour of the blob
 int                 nPts = blobi.nPts;   // number of pts;
 if (nPts > 1){
 nPts = MIN(MAX_N_CONTOUR_POINTS, nPts); 
 for (int j=0; j<nPts; j++){ 
 ofPoint pt = pts[j];
 int px = (int) pt.x; // 0...imgw
 int py = (int) pt.y; // 0...imgh
 cvpts[0][j].x = px;
 cvpts[0][j].y = py;
 }
 ncvpts[0] = nPts;
 cvFillPoly(cvImgTemp, cvpts, ncvpts, 1, color_white, 8, 0 );
 }
 
 }
 ofxCv8uC1_DepthBlobs = cvImgTemp;
 
 unsigned char* currPix = ofxCv8uC1_Depth.getPixels();
 unsigned char* prevPix = ofxCv8uC1_DepthPrev.getPixels();
 unsigned char* maskPix = ofxCv8uC1_DepthBlobs.getPixels();
 int numPixels = ofxCv8uC1_Depth.getWidth() * ofxCv8uC1_Depth.getHeight();
 for(int i = 0; i < numPixels; i++){
 if((currPix[i] == 0) && (prevPix[i] > 0) && (maskPix[i] > 0)){
 currPix[i] = prevPix[i];
 }
 }
 */




/*
 cvAbsDiff(ofxCv8uC1_Depth.getCvImage(), 
 ofxCv8uC1_DepthPrev.getCvImage(), 
 ofxCv8uC1_Temp1.getCvImage());
 
 
 unsigned char* pixA = ofxCv8uC1_Temp1.getPixels();
 unsigned char* pixB = ofxCv8uC1_DepthFrameDif.getPixels();
 for (int i=0; i<(KW*KH); i++){
 pixB[i] = MIN(255, (int)pixA[i] * 8);
 }
 ofxCv8uC1_DepthFrameDif.flagImageChanged();
 ofxCv8uC1_DepthFrameDif.erode_3x3();
 ofxCv8uC1_DepthFrameDif.dilate_3x3();
 
 float A = gui.getValueF("BG_ALPHA", 0);
 float B = 1.0-A;
 float G = gui.getValueF("BG_GAMMA", 0);
 cvAddWeighted(	ofxCv8uC1_DepthBg.getCvImage(), A,
 ofxCv8uC1_DepthFrameDif.getCvImage(),   B, G,
 ofxCv8uC1_Temp1.getCvImage());
 
 
 ofxCv8uC1_DepthBg = ofxCv8uC1_Temp1;
 ofxCv8uC1_DepthBg.blurHeavily();
 */

/*
 // using openCV to find a single maxPoint. Prone to serious jitter. 
 double minVal = 0; 
 double maxVal = 0;
 CvPoint minLoc;
 CvPoint maxLoc;
 cvMinMaxLoc(depthImage.getCvImage(), &minVal, &maxVal, &minLoc, &maxLoc, NULL);
 printf("maxVal = %f , %d, %d\n", (float)maxVal, maxLoc.x, maxLoc.y); 
 */
// ofToString(kinect.getMksAccel().x);