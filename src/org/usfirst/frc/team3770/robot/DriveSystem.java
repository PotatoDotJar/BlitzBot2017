/*
-------------------------------------------------------------------
 BlitzCreek Robotics - FIRST Team 3770
 2016 Season
 DriveSystem Class
 -------------------------------------------------------------------
*/

package org.usfirst.frc.team3770.robot;

import com.ctre.CANTalon;

public class DriveSystem
{
 	public enum DriveChoices {LINEAR, QUADRATIC};
	
    CANTalon rightDriveTalon1;
    CANTalon rightDriveTalon2;
    CANTalon leftDriveTalon1;
    CANTalon leftDriveTalon2;
 	
    DriveChoices driveChoice;

    final double ROTATION_FACTOR = 0.5;

    public DriveSystem(int leftID1, int leftID2, int rightID1, int rightID2, DriveChoices currDrive)
    {   
        rightDriveTalon1 = new CANTalon(rightID1);
        rightDriveTalon2  = new CANTalon(rightID2);
        leftDriveTalon1  = new CANTalon(leftID1);
        leftDriveTalon2  = new CANTalon(leftID2);
        driveChoice = currDrive;
    }
	
 	// Manage left drive wheels
 	public void driveL(double inputL)
	{
    	leftDriveTalon1.set(driveCalc(-inputL));
    	leftDriveTalon2.set(driveCalc(-inputL));
	}
	
 	// Manage right drive wheels
	public void driveR(double inputR)
	{
    	rightDriveTalon1.set(driveCalc(inputR));
    	rightDriveTalon2.set(driveCalc(inputR));
	}
	
    // ----------------------------------------------------------------------------
    // Set drive calculations
    // Options:  LINEAR and QUADRATIC
	public double driveCalc(double input)
	{		
		double output = 0.0;
		
		if (driveChoice == DriveChoices.QUADRATIC)
		{
		    if(input < 0) {
			    output = -input * input;   
		    }
		    if(input > 0) {
			    output = input * input;
		    }
		}
		if (driveChoice == DriveChoices.LINEAR)
		{
			output = input;
		}
		
        return output;
	}
	


}
