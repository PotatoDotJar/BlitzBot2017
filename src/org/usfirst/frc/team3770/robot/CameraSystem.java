// -------------------------------------------------------------------
// BlitzCreek Robotics - FIRST Team 3770
// 2016 Season
// Control front and back viewing cameras
// -------------------------------------------------------------------

package org.usfirst.frc.team3770.robot;

import org.opencv.core.Mat;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraSystem {
	
	public enum Mode {
		FRONT, BACK
	}
	
	private int frontCameraID = 0;
	private int backCameraID = 1;
	
	
	Mode currentMode = Mode.FRONT;
	
	UsbCamera frontCamera;
	UsbCamera backCamera;
	
	CvSink cvSink1;
	CvSink cvSink2;
	CvSource outputStream;
	Mat source;
	
	private boolean serverRunning = true;
	
    public CameraSystem() {
    	frontCamera = CameraServer.getInstance().startAutomaticCapture(frontCameraID);
    	frontCamera.setResolution(160, 120);
    	
    	backCamera = CameraServer.getInstance().startAutomaticCapture(backCameraID);
    	backCamera.setResolution(160, 120);
    	
    	cvSink1 = CameraServer.getInstance().getVideo(frontCamera.getName());
    	cvSink2 = CameraServer.getInstance().getVideo(frontCamera.getName());
        outputStream = CameraServer.getInstance().putVideo("Blur", 160, 120);
        
        source = new Mat();
        //Mat output = new Mat();
        
        // TODO: Make Camera updating threaded
    	/*
        Thread cameraHandle = new Thread(new Runnable() {
			@Override
			public void run() {
				while() {
					
				}
				
			}
		});
        cameraHandle.start();
        */
    }
    public void update() {
    	if(currentMode == Mode.FRONT) {
			cvSink1.grabFrame(source);
			backCamera.free();
		}
		else if (currentMode == Mode.BACK) {
			cvSink2.grabFrame(source);
			frontCamera.free();
		}
		
		
        //Imgproc.cvtColor(source, output, Imgproc.COLOR_RGB2RGBA);
        outputStream.putFrame(source);
    }
    
    
    public void setCamera(Mode mode) {
    	this.currentMode = mode;
	}
    
    public Mode getCurrentCameraView() {
    	return currentMode;
    }
    

}