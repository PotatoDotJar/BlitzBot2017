// -------------------------------------------------------------------
// BlitzCreek Robotics - FIRST Team 3770
// 2016 Season
// Control front and back viewing cameras
// -------------------------------------------------------------------

package org.usfirst.frc.team3770.robot;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraSystem 
{
	private int frontCameraID = 0;
	UsbCamera frontCamera;

    public CameraSystem() 
    {
    	frontCamera = CameraServer.getInstance().startAutomaticCapture(frontCameraID);
    	frontCamera.setResolution(160, 120);
    }
 
}