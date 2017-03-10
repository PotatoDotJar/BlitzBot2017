// =======================================================================
// Team 3770 BlitzCreek - 2017 Robot Code
// Main Driver Module
// =======================================================================

package org.usfirst.frc.team3770.robot;

import org.usfirst.frc.team3770.robot.ActuatorDouble.ActuatorStatus;
import org.usfirst.frc.team3770.robot.CameraSystem;
import org.usfirst.frc.team3770.robot.DriveSystem.DriveChoices;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ================================Dev for final Blitzcreek bot=======================================

public class Robot extends IterativeRobot
{
	// Declare constant values
	private final int LEFT_STICK_USB_PORT    = 1;
	private final int RIGHT_STICK_USB_PORT   = 0;
	private final int CONTROLLER_USB_PORT 	 = 2;

	private final int Left_Motor_1_ID		 = 2;
	private final int Left_Motor_2_ID		 = 4;
	private final int Right_Motor_1_ID		 = 7;
	private final int Right_Motor_2_ID		 = 5;
	
	private final double AUTON_SLOW_THRESHOLD_1 = 0.4;
	private final double AUTON_SLOW_THRESHOLD_2 = 0.6;
	private final double AUTON_STOP_THRESHOLD   = 0.8;

	//private final int VISION_LED_RELAY_PORT  = 0;   

	private final int CLAW_CYLINDER_IN_PORT = 1;
	private final int CLAW_CYLINDER_OUT_PORT = 0;

	private final int CLAW_ROTATE_CYLINDER_IN_PORT = 3;
	private final int CLAW_ROTATE_CYLINDER_OUT_PORT = 2;

	private final int CLAW_ASSEMBLY_CYLINDER_IN_PORT = 5;
	private final int CLAW_ASSEMBLY_CYLINDER_OUT_PORT = 4;

	private final int TRACTION_WHEEL_CYLINDER_IN_PORT = 6;

	private final int LIFTER_RIGHT_Motor 			  = 8;
	private final int LIFTER_LEFT_Motor 			  = 3;

	private final int ANALOG_IR_SENSOR_PORT           = 3;
	private final int LED_DIGITAL_IO_PORT             = 0;

	final int SWITCH_PORT = 0;

	// Declare objects for mechanical units
	DriveSystem drive;
	Joystick leftStick, rightStick, controller;             // Joysticks

	Relay visionLedRelay;                       // Vision light switch

	ActuatorDouble clawCylinder;                // Claw Cylinder
	ActuatorDouble clawRotateCylinder;			// Claw Rotate Cylinder
	ActuatorDouble clawAssemblyCylinder;		// Claw Assembly Cylinder
	Solenoid tractionWheel;

	Lifter lifter;

	Compressor compressor;

	CameraSystem cameraSystem; 					// Manage cameras - front/back

	AnalogInput irDistanceSensor;
	DigitalInput opticalDistanceSensor;
	DigitalOutput clawLight;

	// Timer object(s)
	Timer autonClock, clawOpenDelay, reverseDriveTimer, shimmy, driveShimmy, clawLightTimer;

	Debug debug;                                // Debug Utility Class

	// Declare utility variables

	boolean stoppedAtWall, clawOut, clawOpen, driveStart, driveDone, turning1, turning2, shimmy1, turn;
	boolean isOut, shimmy2, shimmy3, shimmy4, shimmyStop;
	boolean shimmyEngaged = false, shimmy22 = false, shimmy33 = false;
    boolean shimmy44 = false, shimmyStop2 = false;

	int autonRoutine;

	// =======================================================================
	public void robotInit() 
	{	
		// Instantiate robot objects by calling constructors
		drive = new DriveSystem(Left_Motor_1_ID, Left_Motor_2_ID, Right_Motor_1_ID, Right_Motor_2_ID, DriveChoices.QUADRATIC);

		// Create Debug object
		debug = new Debug();

		// Initialize the Joysticks
		leftStick  = new Joystick(LEFT_STICK_USB_PORT);     
		rightStick = new Joystick(RIGHT_STICK_USB_PORT);
		controller = new Joystick(CONTROLLER_USB_PORT);

		// Pneumatics
		clawCylinder = new ActuatorDouble(CLAW_CYLINDER_IN_PORT, CLAW_CYLINDER_OUT_PORT, ActuatorStatus.IN);
		clawAssemblyCylinder = new ActuatorDouble(CLAW_ASSEMBLY_CYLINDER_IN_PORT, CLAW_ASSEMBLY_CYLINDER_OUT_PORT, ActuatorStatus.IN);
		clawRotateCylinder = new ActuatorDouble(CLAW_ROTATE_CYLINDER_IN_PORT, CLAW_ROTATE_CYLINDER_OUT_PORT, ActuatorStatus.OUT);
		tractionWheel = new Solenoid(TRACTION_WHEEL_CYLINDER_IN_PORT);

		lifter = new Lifter(LIFTER_RIGHT_Motor, LIFTER_LEFT_Motor);

		compressor = new Compressor();
		
		irDistanceSensor = new AnalogInput(ANALOG_IR_SENSOR_PORT);
		
		clawLight = new DigitalOutput(LED_DIGITAL_IO_PORT);

		driveShimmy = new Timer();
		clawLightTimer = new Timer();
		
		cameraSystem = new CameraSystem();
  
		/*
        visionLedRelay = new Relay(VISION_LED_RELAY_PORT, Direction.kForward);
        visionLedRelay.set(Value.kOn);
        */
        
		// Various initializatons
		tractionWheel.set(false);
		compressor.enabled();
		
		shimmyEngaged = false;
		shimmy22 = false;
		shimmy33 = false;
		shimmy44 = false;
		shimmyStop2 = false;

		// Clear the dashboard
		debug.clearDashboard();
		System.out.println("=============ROBOT INITIALIZED!=============");;

	}

	// =======================================================================    
	public void autonomousInit()
	{
		autonClock = new Timer();
		clawOpenDelay = new Timer();
		reverseDriveTimer = new Timer();
		shimmy = new Timer();
		autonClock.reset();
		autonClock.start();
		stoppedAtWall = false;
		clawOut = false;
		clawOpen = false;
		driveStart = false;
		driveDone = false;   
		turning1 = false;
		turning2 = false;
		shimmy1 = false;
		shimmy2 = false;
		shimmy3 = false;
		shimmy4 = false;
		shimmyStop = false;
		turn = false;		

		if (SmartDashboard.getBoolean("DB/Button 1",false))
			autonRoutine = 1;   // Left
		else if (SmartDashboard.getBoolean("DB/Button 2",false))
			autonRoutine = 2;   // Center
		if (SmartDashboard.getBoolean("DB/Button 3",false))
			autonRoutine = 3;   // Right
			
	}

	// =======================================================================
	public void autonomousPeriodic()
	{ 
		if(autonRoutine == 2)
		{
			turn = true;
		}
					
		// Left side - Turn Right
		// Routine 1
		if (autonRoutine == 1 && turn == false)
		{
			if (autonClock.get() < 1.6 && stoppedAtWall == false)	
	    	{
	    		drive.driveL(-0.6);
	    		drive.driveR(-0.6);
	    		
	    	}
			else if (autonClock.get() < 2.2 && stoppedAtWall == false)	
	    	{
	    		drive.driveL(-0.72);
	    		drive.driveR(-0.3);
	    	}
			else if (autonClock.get() >= 2.2 && stoppedAtWall == false)	
	    	{
	    		turn = true;
	    	}
		}

		// Right side - Turn Left
		// Routine 3
		else if (autonRoutine == 3 && turn == false)
		{   	 
			if (autonClock.get() < 1.6 && stoppedAtWall == false)	
	    	{
	    		drive.driveL(-0.6);
	    		drive.driveR(-0.6);
	    	}
			else if (autonClock.get() < 2.2 && stoppedAtWall == false)	
	    	{
	    		drive.driveL(-0.3);
	    		drive.driveR(-0.72);
	    	}
			else if (autonClock.get() >= 2.2 && stoppedAtWall == false)	
	    	{
	    		turn = true;
	    	}
		}

		// Go straight
		else if(irDistanceSensor.getAverageVoltage()<= AUTON_SLOW_THRESHOLD_1 && stoppedAtWall == false && turn == true)
		{
			drive.driveL(-0.6);
			drive.driveR(-0.62);
		}
        // First deceleration
		else if (irDistanceSensor.getAverageVoltage()<= AUTON_SLOW_THRESHOLD_2 && stoppedAtWall == false && turn == true)
		{
			drive.driveL(-0.5);
			drive.driveR(-0.51);
		}
        // Second deceleration
		else if (irDistanceSensor.getAverageVoltage()<= AUTON_STOP_THRESHOLD && stoppedAtWall == false && turn == true)
		{
			drive.driveL(-0.4);
			drive.driveR(-0.4);
		}
		// Sensor value exceeds max and indicates to stop
		else if (stoppedAtWall == false)
		{
			drive.driveL(0.0);
			drive.driveR(0.0);
			stoppedAtWall = true;
		}

		// When stopped start timer for claw sequence
		if (stoppedAtWall == true && clawOut == false)
		{
			clawAssemblyCylinder.goOut();
			clawOut = true;
			clawOpenDelay.reset();
			clawOpenDelay.start();
		}
		
		// After delay, start shimmy action
		if (stoppedAtWall == true && clawOut == true && shimmy1 == false && clawOpenDelay.get() > 1.0)
		{
			shimmy.reset();
			shimmy.start();
			shimmy1 = true;
		}
		
		// Shimmy action 1 - Left
		if (shimmy1 == true && shimmy.get() > 0.0 && shimmy2 == false)
		{
			drive.driveL(-0.5);
			drive.driveR(0.2);
			shimmy2 = true;
		}
		
		// Shimmy action 2 - Right
		if (shimmy2 == true && shimmy.get() > 0.3 && shimmy3 == false)
		{
			drive.driveL(0.2);
			drive.driveR(-0.5);
			shimmy3 = true;
		}
		
		// Shimmy action 3 - Brief forward
		if (shimmy3 == true && shimmy.get() > 0.6 && shimmy4 == false)
		{
			drive.driveL(-0.5);
			drive.driveR(-0.5);
			shimmy4 = true;
		}
		
		// Terminate shimmy
		if (shimmy4 == true && shimmy.get() > 1.1 && shimmyStop == false)
		{
			drive.driveL(0.0);
			drive.driveR(0.0);
			shimmyStop = true;
		}

        // After delay for total shimmy action, open claw
		if (clawOpen == false && clawOpenDelay.get() > 3.0)
		{
			clawCylinder.goOut();
			//clawRotateCylinder.goIn();
			clawOpen = true;
		}

		// After delay begin reverse motion
		if (clawOpen == true && driveStart == false && clawOpenDelay.get() > 5.0)
		{	
			//TODO: ADD MAYBE
			//drive.driveL(0.6);
			//drive.driveR(0.6);
			reverseDriveTimer.reset();
			reverseDriveTimer.start();
			driveStart = true;
		}

		// After reverse motion delay, stop
		if (driveDone == false && reverseDriveTimer.get() > 1.0)
		{
			drive.driveL(0.0);
			drive.driveR(0.0);
			driveDone = true;
		}

		debug.print(0, "Analog: "  + irDistanceSensor.getAverageVoltage());
		debug.print(2, "Auton: " + autonRoutine);
	}


	// =======================================================================
	@SuppressWarnings("deprecation")
	boolean driveInterrupted = false;
	public void teleopPeriodic() 
	{
		// Set drive motors to current joy stick values
		if(!driveInterrupted) 
		{
		    if(leftStick.getRawButton(4))
			{
				drive.driveL(leftStick.getY()/2.0);
				drive.driveR(rightStick.getY()/2.0);
			}
		    else if(leftStick.getRawButton(3))
		    {
		    	drive.driveL(-0.5);
		    	drive.driveR(-0.5);
		    }
			else
			{
			    drive.driveL(leftStick.getY());
			    drive.driveR(rightStick.getY());
			}
		}

		// Manage any new control events
		updateControls();
				
		// Manage messaging
		debug.print(1, "Claw: " + clawCylinder.getStatusString());
		
		if(clawRotateCylinder.getStatus() == ActuatorDouble.ActuatorStatus.OUT)
		{
			debug.print(2, "Claw Rot: IN");
		}
		else {
			debug.print(2, "Claw Rot: OUT");
		}
		
		
		debug.print(3, "Claw Assem: " + clawAssemblyCylinder.getStatusString());
		//System.out.println("Running: " + System.currentTimeMillis());
	}

	// =======================================================================    
	public void updateControls()
	{	
		// Drive lifting hook.  If game controller thumb exceeds +0.75 or -0.75,
		// then use stick value to drive motor.  This eliminates noise for stick
		// values near zero.
		if (Math.abs(controller.getY()) > 0.75 )
		{
			lifter.setLiftSpeed(Math.abs(controller.getY()));
		}
		else
		{
			lifter.setLiftSpeed(0);
		}

		// Rotate claw cylinder	
		if(controller.getRawButton(2)) 
		{
			clawRotateCylinder.goIn();
		}
		else if(controller.getRawButton(4))
		{
			clawRotateCylinder.goOut();
		}        	

		// Engage claw
		if(controller.getRawAxis(3) > 0) 
		{
			clawCylinder.goOut();
			
		}
		else if(controller.getRawAxis(2) > 0) 
		{
			clawCylinder.goIn();
		}

		// Manage claw assembly
		if(controller.getRawButton(3)) 
		{
			clawAssemblyCylinder.goOut();
		}
		else if(controller.getRawButton(1)) 
		{
			clawAssemblyCylinder.goIn();
		}   	

        // Traction wheel engage
		if(leftStick.getRawButton(1)) 
		{
			tractionWheel.set(true);
		}
		else if(!leftStick.getRawButton(1)) 
		{
			tractionWheel.set(false);
		}

		// Set or unset reversed drive    	
		if (rightStick.getRawButton(12))
		{
			drive.setReversed(true);
		}
		else if (rightStick.getRawButton(11))
		{
			drive.setReversed(false);
		}

		// Shimmy Stuff
		if(rightStick.getRawButton(1) && shimmyEngaged == false)
		{
			shimmyEngaged = true;
			driveInterrupted = true;
			driveShimmy.reset();
			driveShimmy.start();
		}
		
		if (shimmyEngaged)
			manageShimmy();
	}
	
	// =======================================================================   
	// Method to manage ongoing shimmy action
	public void manageShimmy()
	{
		if (shimmy22 == false)
		{
			drive.driveL(-0.5);
			drive.driveR(0.5);
			if(driveShimmy.get() > 0.3) {
				shimmy22 = true;
			}
			
		}
		else if (shimmy22 == true && shimmy33 == false)
		{
			drive.driveL(0.5);
			drive.driveR(-0.5);
			if(driveShimmy.get() > 0.6) {
				shimmy33 = true;
			}
		}
		else if (shimmy33 == true && shimmy44 == false)
		{
			drive.driveL(-0.5);
			drive.driveR(-0.5);
			if(driveShimmy.get() > 1.1) {
				shimmy44 = true;
			}
		}
		else if (shimmy44 == true && shimmyStop2 == false)
		{
			drive.driveL(0.0);
			drive.driveR(0.0);
			shimmyEngaged = false;
			shimmy22 = false;
			shimmy33 = false;
			shimmy44 = false;
			shimmyStop2 = false;
			driveInterrupted = false;

		}
		
	}

}