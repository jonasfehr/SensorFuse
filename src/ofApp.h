#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxGUI.h"
#include "GateSF.h"
#include "SoundObject.h"
#include "MsaPhysics2D.h"
#include "ActivityMap.h"
#include "DistributionGen.h"
#include "defines.h"



class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void setupGUI();
    
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
    
    //amount of time until message fades
    float fadeTime;
    
    ofxOscReceiver	receiver;
    ofxOscSender senderVisual;
    ofxOscSender senderAudio;
    ofxOscReceiver    receiverFlocking;
    ofxOscSender senderFlocking;
    
    int				current_msg_string;
    string          msg_strings[NUM_MSG_STRINGS];
    float			timers[NUM_MSG_STRINGS];
    
    //OSC address split
    vector<string>  msgTokens;
    
    //list of artnetAddress - todo: load off xml file?
    //need to be ordered asc
    vector<int>  artnetAddrs;
    
    std::map<int,GateSF> gates;
    
    //display history of setting for debuging debouncing
    //x = sensorValue, y = triggerConfidence
    vector<ofVec2f> gateDisplay;

    // Sound objects moving around
    vector<SoundObject> soundObjects;
    Particle2D_ptr idleParticle;

    // Stuff from positionEstimator
    World2D_ptr world;
    vector<User> users;
    
    // GUI
    ofParameterGroup guiParameters;
    ofxPanel gui;
    ofxPanel guiActivity;
    ofxPanel guiDistribution;

    bool hideGui = false;
    ofParameter<int> timingThreshold;
    ofParameter<float> distanceThreshold;
    ofParameter<int> debounceLower;
    ofParameter<int> debounceHigher;
    ofParameter<bool> drawGatesToggle;
    ofParameter<bool> drawUsersToggle;
    ofParameter<bool> drawSoundObjects;
    ofParameter<bool> drawActivityMap;
    ofParameter<bool> drawFlowField;
    ofParameter<bool> drawFlock;
    ofParameter<bool> doUsersAttract;
    ofParameter<float> targetAvgVelocity;
    ofParameter<bool> simulation;
    ofParameter<float> possibility;


    int oldMillis = 0;
    
    // Axtivity measure
    ActivityMap activityMap;
    
    DistributionMap distributionMap;
    
    bool sendFlag = true;
    
    ofxSyphonServer syphonOutSoundObjects;
    ofFloatImage imgSoundObjects;

};


