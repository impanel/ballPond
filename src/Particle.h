/*
 *  Particle.h
 *  kinectBallPit
 *
 *  Created by impanel on 7/3/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ofxBox2d.h"
#include "ofMain.h"

#ifndef PARTICLE_H
#define PARTICLE_H

class Particle : public ofxBox2dCircle {
	
public:
	
	//Particle() 	
	
	ofColor color;
	int r;
	int g;
	int b;
	int groupId;
	void draw();
	void update();
    
};
#endif