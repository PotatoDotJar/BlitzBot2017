// -------------------------------------------------------------------
// BlitzCreek Robotics - FIRST Team 3770
// 2016 Season
// Control solenoid for one actuator action - Double solenoid
// -------------------------------------------------------------------

package org.usfirst.frc.team3770.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;

public class ActuatorDouble 
{
	public enum ActuatorStatus {IN,OUT}; 
	
    DoubleSolenoid controlSolenoid;
    
    ActuatorStatus status;
    String statusString;
    int inPort;
    int outPort;

	// --------------------------------------------------------------------------------------
    // Default constructor
    public ActuatorDouble(int inPort, int outPort, ActuatorStatus initStatus) 
    {
    	this.inPort  = inPort;
    	this.outPort = outPort;
    	controlSolenoid = new DoubleSolenoid(inPort,outPort);
    	statusString = new String();
        status = initStatus;
        
        if(status == ActuatorStatus.IN) {
        	goIn();
        }else{
        	goOut();
        }
    }

    // Activate cylinder - Out
    public void goOut()
    {
    	controlSolenoid.set(DoubleSolenoid.Value.kForward);
        status = ActuatorStatus.OUT;
    }

    // Activate cylinder - In
    public void goIn()
    {
    	controlSolenoid.set(DoubleSolenoid.Value.kReverse);
        status = ActuatorStatus.IN;
    }
    /*
    // Manage actuators in motion.  Shut down with time expires.
    public void manageActions()
    {
    	// Manage actions in progress
        if (actuatorMoving && actuatorDelay.hasPeriodPassed(solenoidTimer))
        {
        	controlSolenoid.set(DoubleSolenoid.Value.kOff);
        	actuatorDelay.stop();
        	actuatorDelay.reset();
        }        
    }
    */
    public ActuatorStatus getStatus()
    {
    	 return status;
    }
   
    public String getStatusString()
    {
    	String outMsg = new String();
    	 if (status == ActuatorStatus.IN)
    		 outMsg = "IN";
    	 if (status == ActuatorStatus.OUT)
    		 outMsg = "OUT";
    	 return outMsg;
     }
}
