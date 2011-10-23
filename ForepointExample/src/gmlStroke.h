/*
 *  gmlStroke.h
 *  based on ofxCvBlob.h
 *
 */


#ifndef GML_STROKE_H
#define GML_STROKE_H


#include "ofMain.h"


class gmlStroke {
	
public:
	
	vector <ofPoint>    pts;    // the points
	int                 nPts;   // number of pts;
	
	//----------------------------------------
	gmlStroke() {
		nPts = 0;
	}
	
	//----------------------------------------
	void draw(){
		
		ofNoFill();
		
		glColor4f(1,1,1, 0.5);
		glLineWidth (4.0);
		ofEnableAlphaBlending();
		glEnable(GL_LINE_SMOOTH);
		glBegin(GL_LINE_STRIP);
		
		nPts = pts.size();
		for (int i = 0; i < nPts; i++){
			glVertex2f (pts[i].x, pts[i].y);
		}
		glEnd();
		
		glLineWidth (1.0);
		glDisable(GL_LINE_SMOOTH);
		ofDisableAlphaBlending();
	}
	
	//----------------------------------------
	void add (float x, float y, float z){
		pts.push_back( ofPoint(x, y, z) );
		nPts = pts.size();
	}
	
	//----------------------------------------
	void clear(){
		pts.clear();
		nPts = 0;
	}
	
	
};


#endif


