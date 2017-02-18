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
    
    boolean reversedDrive;

    final double ROTATION_FACTOR = 0.5;

    public DriveSystem(int leftID1, int leftID2, int rightID1, int rightID2, DriveChoices currDrive)
    {   
        rightDriveTalon1 = new CANTalon(rightID1);
        rightDriveTalon2  = new CANTalon(rightID2);
        leftDriveTalon1  = new CANTalon(leftID1);
        leftDriveTalon2  = new CANTalon(leftID2);
        driveChoice = currDrive;
        
        reversedDrive = false;
    }
	
 	// Manage left drive wheels
    // Reversed drive allows backwards driving from a forward reference.
 	public void driveL(double inputL)
	{
 		if(reversedDrive == false)
 		{
 			leftDriveTalon1.set(driveCalc(-inputL));
 			leftDriveTalon2.set(driveCalc(-inputL));
 		}
 		else
 		{
 			rightDriveTalon1.set(driveCalc(-inputL));
 			rightDriveTalon2.set(driveCalc(-inputL));
 		}
 	}
	
 	// Manage right drive wheels
    // Reversed drive allows backwards driving from a forward reference.
	public void driveR(double inputR)
	{
 		if(reversedDrive == false)
 		{
 			rightDriveTalon1.set(driveCalc(inputR));
 			rightDriveTalon2.set(driveCalc(inputR));
 		}
 		else
 		{
 			leftDriveTalon1.set(driveCalc(inputR));
 			leftDriveTalon2.set(driveCalc(inputR));
 		}
	}
	
	// Manage reverse drive
	public void setReversed(boolean r)
	{
		reversedDrive = r;
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
		else if (driveChoice == DriveChoices.LINEAR)
		{
			output = input;
		}
		
		
        return output;
	}
	


}
