#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    // listen on the given port
    cout << "listening for osc messages on port " << PORT << "\n";
    receiver.setup( PORT );
    
    current_msg_string = 0;
    mouseX = 0;
    mouseY = 0;
    mouseButtonState = "";
    
    ofBackground( 30, 30, 130 );
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
    // hide old messages
    for ( int i=0; i<NUM_MSG_STRINGS; i++ )
    {
        if ( timers[i] < ofGetElapsedTimef() )
            msg_strings[i] = "";
    }
    
    // check for waiting messages
    while( receiver.hasWaitingMessages() )
    {
        // get the next message
        ofxOscMessage m;
        receiver.getNextMessage( &m );
        
        // check for mouse moved message
        if ( m.getAddress() == "/mouse/position" )
        {
            // both the arguments are int32's
            mouseX = m.getArgAsInt32( 0 );
            mouseY = m.getArgAsInt32( 1 );
        }
        // check for mouse button message
        else if ( m.getAddress() == "/mouse/button" )
        {
            // the single argument is a string
            mouseButtonState = m.getArgAsString( 0 ) ;
        }
        else if ( m.getAddress() == "/BeamBreak/016" ){
            
            string msg_string = "beamBreak!";
            
            //msg_string += ofToString( m.getArgAsInt32( i ) );
            
            
            
            msg_strings[current_msg_string] = msg_string;
            current_msg_string = ( current_msg_string + 1 ) % NUM_MSG_STRINGS;
            // clear the next line
            msg_strings[current_msg_string] = "";

            
        }
        else
        {
            // unrecognized message: display on the bottom of the screen
            string msg_string;
            msg_string = m.getAddress();
            msg_string += ": ";
            for ( int i=0; i<m.getNumArgs(); i++ )
            {
                // get the argument type
                msg_string += m.getArgTypeName( i );
                msg_string += ":";
                // display the argument - make sure we get the right type
                if( m.getArgType( i ) == OFXOSC_TYPE_INT32 ){
                    msg_string += ofToString( m.getArgAsInt32( i ) );
                    msg_string += " int";
                }
                else if( m.getArgType( i ) == OFXOSC_TYPE_FLOAT ){
                    msg_string += ofToString( m.getArgAsFloat( i ) );
                    msg_string += " float";
                }
                else if( m.getArgType( i ) == OFXOSC_TYPE_STRING ){
                    msg_string += m.getArgAsString( i );
                    msg_string += " string";
                }
                else
                    msg_string += "unknown";
            }
            // add to the list of strings to display
            msg_strings[current_msg_string] = msg_string;
            timers[current_msg_string] = ofGetElapsedTimef() + 5.0f;
            current_msg_string = ( current_msg_string + 1 ) % NUM_MSG_STRINGS;
            // clear the next line
            msg_strings[current_msg_string] = "";
        }
        
    }

   
}

//--------------------------------------------------------------
void ofApp::draw(){
/*
    ofTranslate( ofGetWidth()/2, ofGetHeight()/2);
    ofDrawCircle(oscX, oscY, oscZ+10);
    ofDrawBitmapString(messageText, 30, 30);
    ofDrawBitmapString(color, 30, 40);
    
*/
    
    string buf;
    buf = "listening for osc messages on port" + ofToString( PORT );
    ofDrawBitmapString( buf, 10, 20 );
    
    // draw mouse state
    buf = "mouse: " + ofToString( mouseX, 4) +  " " + ofToString( mouseY, 4 );
    ofDrawBitmapString( buf, 430, 20 );
    ofDrawBitmapString( mouseButtonState, 580, 20 );
    
    for ( int i=0; i<NUM_MSG_STRINGS; i++ )
    {
        ofDrawBitmapString( msg_strings[i], 10, 40+15*i );
    }


}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
