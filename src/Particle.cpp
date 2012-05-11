/*
 *  Particle.cpp
 *  kinectBallPit
 *
 *  Created by impanel on 7/3/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Particle.h"

//--------------------------------------------------------------
void Particle::draw()
{
	ofSetColor( r, g, b );
	ofCircle( getPosition().x, getPosition().y, getRadius() );
}

//--------------------------------------------------------------
void Particle::update()
{
	if(getPosition().x > ofGetWidth() || getPosition().x < 0)
	{
		setPosition(ofRandom( 0, ofGetWidth() ), ofRandom( ofGetHeight()-150, ofGetHeight() ));
	}
	if(getPosition().y > ofGetHeight() || getPosition().y < 0)
	{
		setPosition(ofRandom( 0, ofGetWidth() ), ofRandom( ofGetHeight()-150, ofGetHeight() ));
	}
}

