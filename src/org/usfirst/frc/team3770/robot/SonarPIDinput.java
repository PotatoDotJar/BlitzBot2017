package org.usfirst.frc.team3770.robot;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.AnalogInput;

class SonarPIDinput implements PIDSource
{
	AnalogInput analogInValue;
	PIDSourceType sourceType;
	
	SonarPIDinput(AnalogInput anIn)
	{
		analogInValue = anIn;
	}

    // Get the result to use in PIDController.
    public double pidGet()
    {
    	double voltage = analogInValue.getVoltage();
        return convertVoltsInches(voltage) ;
    }
    
    
    // Get which parameter of the device you are using as a process control variable.
    public PIDSourceType getPIDSourceType()
    {
    	return sourceType;
    }

    // Set which parameter of the device you are using as a process control variable.
    public void setPIDSourceType(PIDSourceType pidSource)
    {
    	sourceType = pidSource;
    }
    
    
    
    //--------------------------------------------------------------------------------
     // Function to convert sonic rangefinder voltage to inches
     // Derived and modeled from empirical data 
     double convertVoltsInches(double volts) 
     {
         return 102.542 * volts + 1.041;
     }
}