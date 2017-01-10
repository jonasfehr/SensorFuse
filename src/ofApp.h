#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "Sensor.h"
#include "GateSF.h"
#include <array>

#define PORT 49161
#define NUM_MSG_STRINGS 30
#define DEBUG 1

#define SENDHOST "localhost"
#define SENDPORT 49161


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
    
        //todo
        void drawSensor(Sensor sensor);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        ofTrueTypeFont		font;
    
    // Gates
    //vector<ofVec3f> gates;
    
    
    //amount of time until message fades
    float fadeTime;
    
    
    //Geo data
    //ofVec3f gate1Geo = ofVec3f(-50,1,0);

    
  
    ofxOscReceiver	receiver;
    ofxOscSender sender;
    
    int				current_msg_string;
    string          msg_strings[NUM_MSG_STRINGS];
    float			timers[NUM_MSG_STRINGS];
    
    
    //OSC address split
    vector<string>  msgTokens;
    
    
    vector<string>  artnetAddrs{"000", "016", "032"}; //list of artnetAddress


    vector<int> gateDisplay;

    
    //const int lastArtNetAddr = 32;
    //vector<Sensor>  sensors;
    //using array is faster?
    //Sensor sensors[32];
    vector<Sensor> sensors;
    
    
    
};


