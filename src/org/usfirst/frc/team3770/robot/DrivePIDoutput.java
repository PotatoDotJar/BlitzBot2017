package org.usfirst.frc.team3770.robot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.PIDOutput;

class DrivePIDoutput implements PIDOutput
{
	Talon leftMotor;
	Talon rightMotor;
	
    public DrivePIDoutput(Talon left, Talon right)
	{
    	leftMotor  = left;
    	rightMotor = right;
	}

    // Set the output to the value calculated by PIDController.
    public void pidWrite(double output)
    {
    	leftMotor.set(output);
    	rightMotor.set(output);
    }

}