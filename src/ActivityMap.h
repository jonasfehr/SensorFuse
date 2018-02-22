//
//  ActivityMap.h
//  SensorFuse
//
//  Created by Jonas Fehr on 01/02/2018.
//

#pragma once

#include "ofxSyphon.h"

#define NUM_INTERVALS 100

class ActivityAccumulator{
public:
    unsigned long timeIntervals [NUM_INTERVALS];
    unsigned long timeLastTrigger = 0;
    
    int timewindow_slow = 30;
    int timewindow_mid = 30;
    int timewindow_fast = 30;
    float fastRetrigger = 2;
    
    int numTriggers_slow = 0;
    int numTriggers_mid = 0;
    int numTriggers_fast = 0;
    
    float val_slow = 0;
    float val_mid = 0;
    float val_fast = 0;
    
    float val_max = 0;
    
    float damping;
    
    ActivityAccumulator(){
        for(int i = 0; i < NUM_INTERVALS; i++){
            timeIntervals[i] = 100000000;
        }
    };
    
    void trigger(uint64_t elapsedTime){
        
        for(int i = NUM_INTERVALS-1; i > 0; i--){
            timeIntervals[i] = timeIntervals[i-1];
        }
        timeIntervals[0] = elapsedTime-timeLastTrigger;
        timeLastTrigger = elapsedTime;
    }
    
    void update(){
        update(ofGetElapsedTimeMillis(), ofGetFrameRate());
    }
    
    void update(uint64_t elapsedTime, int frameRate){
        
        numTriggers_slow = 0;
        numTriggers_mid = 0;
        numTriggers_fast = 0;
        
        unsigned long timeSum = elapsedTime-timeLastTrigger;
        for(auto & tI : timeIntervals){
            timeSum += tI;
            
            if(timeSum < timewindow_slow*1000){
                numTriggers_slow++;
            }
            if(timeSum < timewindow_mid*1000){
                numTriggers_mid++;
            }
            if(timeSum < timewindow_fast*1000){
                numTriggers_fast++;
            }
        }
        
        val_slow = (val_slow * (1.-damping)) + (ofMap(numTriggers_slow, 0, timewindow_slow/fastRetrigger, 0., 1., true) * damping);
        val_mid = (val_mid * (1.-damping)) + (ofMap(numTriggers_mid, 0, timewindow_mid/fastRetrigger, 0., 1., true) * damping);
        val_fast = (val_fast * (1.-damping)) + (ofMap(numTriggers_fast, 0, timewindow_fast/fastRetrigger, 0., 1., true) * damping);
        val_max = glm::max(val_slow, glm::max(val_mid, val_fast));
    }
};


class ActivityMap{
public:
    string name;
    
    std::map<int,ActivityAccumulator> activity;
    
    ofFloatImage imgActivity;
    ofxSyphonServer syphonOut;
    
    ofParameterGroup parameterGroup;
    ofParameter<bool> deactivate;
    ofParameter<float> timewindow_slow;
    ofParameter<float> timewindow_mid;
    ofParameter<float> timewindow_fast;
    ofParameter<float> fastRetrigger;
    ofParameter<float> damping;

    vector<int> addresses;
    
    ActivityMap(){};
    
    void setup(string name, vector<int> addresses){
        this->name = name;
        this->activity = activity;
        this->addresses = addresses;
        
        syphonOut.setName(name);
        
        setupParameterGroup(name);
        
        imgActivity.allocate(40, 3, OF_IMAGE_COLOR_ALPHA);
        imgActivity.setColor(ofFloatColor(0.,0.));
        
        for(auto& addr : addresses){
            // Create gate
            activity[addr] = ActivityAccumulator();
        }
            
        
    }
    
    void update(){
        
        uint64_t elapsedTime = ofGetElapsedTimeMillis();
        int frameRate = ofGetFrameRate();
        
        int i = 0;
        for(auto& a : activity){
            a.second.timewindow_slow = timewindow_slow;
            a.second.timewindow_mid = timewindow_mid;
            a.second.timewindow_fast = timewindow_fast;
            a.second.fastRetrigger = fastRetrigger;
            a.second.damping = damping;

            a.second.update(elapsedTime, frameRate);
            

            imgActivity.setColor(i, 1, ofFloatColor(a.second.val_slow, a.second.val_mid, a.second.val_fast , a.second.val_max));
//            imgActivity.setColor(i, 1, ofFloatColor(0. ,  a.second.val_fast));
            i++;
        }
        
        if(deactivate) imgActivity.setColor(ofFloatColor(0.,0.));
        imgActivity.update();
        
    }
    
    
    
    void publish(){
        // PUBLISH OUTPUT
        ofSetColor(255);
        ofFill();
        syphonOut.publishTexture(&imgActivity.getTexture());
    }
    
    ofTexture& getTexture(){ return imgActivity.getTexture(); }
    
    
    void draw(int x, int y){
        ofSetColor(255);
        imgActivity.draw(x, y);
    }
    void draw(int x, int y, int w, int h){
        ofSetColor(255);
        imgActivity.draw(x,y,w,h);
    }
    
    void setupParameterGroup(string name){
        
        parameterGroup.setName(name);
        parameterGroup.add(deactivate.set("deactivate", false));
        parameterGroup.add(timewindow_slow.set("timewindow_slow", 30, 1, 600));
        parameterGroup.add(timewindow_mid.set("timewindow_mid", 15, 1, 600));
        parameterGroup.add(timewindow_fast.set("timewindow_fast", 5, 1, 600));
        parameterGroup.add(fastRetrigger.set("fastRetrigger", 4, 0, 10));
        parameterGroup.add(damping.set("damping", 0.1, 0, 1));

    }
    
    ofParameterGroup& getParameterGroup(){ return parameterGroup; }
    string& getName(){ return name; }
    
    ActivityAccumulator getActivity(int index){
        return activity.at(addresses[index]);
    }
private:
    
};

