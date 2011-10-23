/*
 *  holeFilling.cpp
 *  ofxKinect
 *
 *  Created by Golan Levin on 12/5/10.
 *  Copyright 2010 Carnegie Mellon University. All rights reserved.
 *
 */

#include "testApp.h"



//--------------------------------------------------------------
void testApp::fillHolesPre(){
	// Copy the current depth into the previous depth buffer(s)
	cvCopy(ofxCv8uC1_DepthPrev3.getCvImage(), ofxCv8uC1_DepthPrev4.getCvImage(), NULL);
	cvCopy(ofxCv8uC1_DepthPrev2.getCvImage(), ofxCv8uC1_DepthPrev3.getCvImage(), NULL);
	cvCopy(ofxCv8uC1_DepthPrev1.getCvImage(), ofxCv8uC1_DepthPrev2.getCvImage(), NULL);
	cvCopy(ofxCv8uC1_DepthRawThreshed.getCvImage(), ofxCv8uC1_DepthPrev1.getCvImage(), NULL);
}

//--------------------------------------------------------------
void testApp::fillHolesPost(){
	// Dilate the thresholded depth image to clean up small spatial holes.
	int nDepthDilate = gui.getValueI("N_DILATIONS", 0); 
	for (int i=0; i<nDepthDilate; i++){
		ofxCv8uC1_DepthRawThreshed.dilate();
	}
	
	// Fill temporal holes by taking the max of the past N depth frames
	int nDepthHistory = gui.getValueI("N_HISTORY", 0); 
	bool bUseMedian = gui.getValueB("DO_DEPTH_MEDIAN", 0);	
	
	if ((nDepthHistory == 0) && (bUseMedian == false)) {
		ofxCv8uC1_Depth = ofxCv8uC1_DepthRawThreshed;
		
	} else {
		
		
		if (bUseMedian){
			
			int medianSize = 5; // 3 or 5
			switch (medianSize) {
					
				default:
				case 3:
					// Do median of 3
					nDepthHistory = 3;
					gui.setValueI("N_HISTORY", nDepthHistory, 0);
					
					// Max of (A,B) in Temp1
					cvMax(ofxCv8uC1_DepthRawThreshed.getCvImage(), 
						  ofxCv8uC1_DepthPrev1.getCvImage(), 
						  ofxCv8uC1_Temp1.getCvImage());
					
					// Max of (A,C) in Temp2
					cvMax(ofxCv8uC1_DepthRawThreshed.getCvImage(), 
						  ofxCv8uC1_DepthPrev2.getCvImage(), 
						  ofxCv8uC1_Temp2.getCvImage());
					
					// Min of (Temp1, Temp2) in Depth
					cvMin(ofxCv8uC1_Temp1.getCvImage(), 
						  ofxCv8uC1_Temp2.getCvImage(), 
						  ofxCv8uC1_Depth.getCvImage());
					
					break;
			
				//---------------------------------------------
				case 5:
					// Do median of 5
					nDepthHistory = 4;
					gui.setValueI("N_HISTORY", nDepthHistory, 0);
					
					
					unsigned char* pix0		= ofxCv8uC1_DepthRawThreshed.getPixels();
					unsigned char* pix1		= ofxCv8uC1_DepthPrev1.getPixels();
					unsigned char* pix2 	= ofxCv8uC1_DepthPrev2.getPixels();
					unsigned char* pix3 	= ofxCv8uC1_DepthPrev3.getPixels();
					unsigned char* pix4 	= ofxCv8uC1_DepthPrev4.getPixels();
					unsigned char* currPix	= ofxCv8uC1_Depth.getPixels();
					
					int count = 0; 
					int numPixels = KW*KH;
					for (int i = 0; i < numPixels; i++){
						unsigned char  a0 = pix0[i];
						unsigned char  a1 = pix1[i];
						unsigned char  a2 = pix2[i];
						
						if ((a0>0) || (a1>0) || (a2>0)){
							
							unsigned char  a3 = pix3[i];
							unsigned char  a4 = pix4[i];
							
							if (a1 < a0) 	swap(a0, a1);
							if (a2 < a0)	swap(a0, a2); 
							if (a3 < a0)	swap(a0, a3);
							if (a4 < a0)	swap(a0, a4);
							if (a2 < a1)	swap(a1, a2);
							if (a3 < a1) 	swap(a1, a3);
							if (a4 < a1)	swap(a1, a4);
							if (a3 < a2)	swap(a2, a3);
							if (a4 < a2)	swap(a2, a4);
							currPix[i] = a2;
							
						} else {
							currPix[i] = 0;
						}
					}
					break;
			}
			
			
			
		} else {
			// The simplest design:
			// just use the Max of all depth history images instead.
			if (nDepthHistory > 0){
				cvMax(ofxCv8uC1_DepthRawThreshed.getCvImage(), 
					  ofxCv8uC1_DepthPrev1.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
			if (nDepthHistory > 1){
				cvMax(ofxCv8uC1_Depth.getCvImage(), 
					  ofxCv8uC1_DepthPrev2.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
			if (nDepthHistory > 2){
				cvMax(ofxCv8uC1_Depth.getCvImage(), 
					  ofxCv8uC1_DepthPrev3.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
			if (nDepthHistory > 3){
				cvMax(ofxCv8uC1_Depth.getCvImage(), 
					  ofxCv8uC1_DepthPrev4.getCvImage(), 
					  ofxCv8uC1_Depth.getCvImage());
			}
		}
	}
	ofxCv8uC1_Depth.flagImageChanged();
}

