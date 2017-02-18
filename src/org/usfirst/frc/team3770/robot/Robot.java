// =======================================================================
// Team 3770 BlitzCreek - 2017 Robot Code
// Main Driver Module
// =======================================================================

package org.usfirst.frc.team3770.robot;

import org.usfirst.frc.team3770.robot.ActuatorDouble.ActuatorStatus;
import org.usfirst.frc.team3770.robot.CameraSystem.Mode;
import org.usfirst.frc.team3770.robot.DriveSystem.DriveChoices;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
    
    private final int VISION_LED_RELAY_PORT  = 0;   

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
    private final int DIGITAL_OPTICAL_SENSOR_PORT     = 0;
    
    
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
    
    // Timer object(s)
    Timer autonClock, clawOpenDelay, reverseDriveTimer, shimmy, driveShimmy;
    
    Debug debug;                                // Debug Utility Class
    
    // Declare utility variables
    
    boolean stoppedAtWall, clawOut, clawOpen, driveStart, driveDone, turning1, turning2, shimmy1;
    boolean isOut, shimmy2, shimmy3, shimmy4, shimmyStop;
    
    double speedControl;
    
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
        
        // Initializer various objects
        //sonar = new AnalogInput(SONAR_ANALOG_PORT);
        
        irDistanceSensor = new AnalogInput(ANALOG_IR_SENSOR_PORT);
        opticalDistanceSensor = new DigitalInput(DIGITAL_OPTICAL_SENSOR_PORT);
        
        driveShimmy = new Timer();
        
        //cylinder = new ActuatorDouble(CYLINDER_IN_PORT, CYLINDER_OUT_PORT, ActuatorStatus.IN);
        //distanceTrigger = new DigitalInput(DIGITAL_DISTANC_SENSOR_PORT);
   /*     
        visionLedRelay = new Relay(VISION_LED_RELAY_PORT, Direction.kForward);
        visionLedRelay.set(Value.kOn);
        cameraSystem = new CameraSystem();
        */
        tractionWheel.set(false);
        compressor.enabled();
        
        // Clear the dashboard
        debug.clearDashboard();
        System.out.println("=============ROBOT INITIALIZED!=============");
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
    	
    	//if (SmartDashboard.getBoolean("DB/Button 1")) 
    }
    
    // =======================================================================
    public void autonomousPeriodic()
    { 
	/*

		// turn right
		 
		 if (autonClock.get() < 1.4 && stoppedAtWall == false)	
    	{
    		drive.driveL(-0.6);
    		drive.driveR(-0.6);
    	}
		else	if (autonClock.get() < 2.2 && stoppedAtWall == false)	
    	{
    		drive.driveL(-0.6);
    		drive.driveR(-0.3);
    	}
    	
    	//turn left
    	
    	 if (autonClock.get() < 1.4 && stoppedAtWall == false)	
    	{
    		drive.driveL(-0.6);
    		drive.driveR(-0.6);
    	}
		else	if (autonClock.get() < 2.2 && stoppedAtWall == false)	
    	{
    		drive.driveL(-0.3);
    		drive.driveR(-0.6);
    	}
    */
    	
    	
    	// Successful Auton starts here
    	
    	//go straight

		if(irDistanceSensor.getAverageVoltage()<=.4 && stoppedAtWall == false)
    	{
    		drive.driveL(-0.6);
    		drive.driveR(-0.6);
    	}

    	else if (irDistanceSensor.getAverageVoltage()<=.6 && stoppedAtWall == false)
    	{
    		drive.driveL(-0.5);
    		drive.driveR(-0.5);
    	}

    	else if (irDistanceSensor.getAverageVoltage()<=.8 && stoppedAtWall == false)
    	{
    		drive.driveL(-0.4);
    		drive.driveR(-0.4);
    	}
    	else if (stoppedAtWall == false)
    	{
    		drive.driveL(0.0);
    		drive.driveR(0.0);
    		stoppedAtWall = true;
    	}
		
		if (stoppedAtWall == true && clawOut == false)
		{
			clawAssemblyCylinder.goOut();
			clawOut = true;
			clawOpenDelay.reset();
			clawOpenDelay.start();
		}
		if (stoppedAtWall == true && clawOut == true && shimmy1 == false && clawOpenDelay.get() > 1.0)
		{
			shimmy.reset();
			shimmy.start();
			shimmy1 = true;
		}
//===================================================================================================
		if (shimmy1 == true && shimmy.get() > 0.0 && shimmy2 == false)
		{
			drive.driveL(-0.5);
			drive.driveR(0.5);
			shimmy2 = true;
		}
		if (shimmy2 == true && shimmy.get() > 0.3 && shimmy3 == false)
		{
			drive.driveL(0.5);
			drive.driveR(-0.5);
			shimmy3 = true;
		}
		if (shimmy3 == true && shimmy.get() > 0.6 && shimmy4 == false)
		{
			drive.driveL(-0.5);
			drive.driveR(-0.5);
			shimmy4 = true;
		}
		if (shimmy4 == true && shimmy.get() > 1.1 && shimmyStop == false)
		{
			drive.driveL(0.0);
			drive.driveR(0.0);
			shimmyStop = true;
		}
//==================================================================================================
		if (clawOpen == false && clawOpenDelay.get() > 3.0)
		{
			clawCylinder.goOut();
			clawRotateCylinder.goIn();
			clawOpen = true;
		}
		
		if (clawOpen == true && driveStart == false && clawOpenDelay.get() > 5.0)
		{	
    		drive.driveL(0.6);
    		drive.driveR(0.6);
    		reverseDriveTimer.reset();
    		reverseDriveTimer.start();
    		driveStart = true;
		}
		
		if (driveDone == false && reverseDriveTimer.get() > 1.0)
		{
    		drive.driveL(0.0);
    		drive.driveR(0.0);
    		driveDone = true;
		}
	
    	

	
    debug.print(0, "Analog: "  + irDistanceSensor.getAverageVoltage());
    debug.print(1, "Digital: " + opticalDistanceSensor.get());
    	
}
  
    
    // =======================================================================
    public void teleopPeriodic() 
    {
    	shimmy1 = false;
    	shimmy2 = false;
    	shimmy3 = false;
    	shimmy4 = false;
    	shimmyStop = false;

        // Set drive motors to current joy stick values
        drive.driveL(leftStick.getY());
      drive.driveR(rightStick.getY());
        
        speedControl = SmartDashboard.getNumber("DB/Slider 0");
        
        
        
        // Manage any new control events
    	updateControls();
       // cameraSystem.update();
        debug.print(0, "Lifter Speed: " + speedControl);
        debug.print(1, "Claw: " + clawCylinder.getStatusString());
        debug.print(2, "Claw Rot: " + clawRotateCylinder.getStatusString());
        debug.print(3, "Claw Assem: " + clawAssemblyCylinder.getStatusString());
        debug.print(4, "Com A: " + compressor.getCompressorCurrent());
        debug.print(5, "Climber A: " + (lifter.getLeftCurrent()+lifter.getRightCurrent()));
        
        debug.print(6, "Analog: "  + irDistanceSensor.getAverageVoltage());
        debug.print(7, "Digital: " + opticalDistanceSensor.get());

        
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
    	
 /*      	 if(compressor.getClosedLoopControl()) {
        		compressor.stop(); 
        	}
    	 }
        	
        

        	if(!compressor.getClosedLoopControl()) {
        		compressor.enabled();
        	}
        }
        */
    	
    	 /*
    	// Switch Cameras
    	if (leftStick.getRawButton(11)) {
    		cameraSystem.setCamera(Mode.BACK);
    	}
    	else if (leftStick.getRawButton(12)) {
    		cameraSystem.setCamera(Mode.FRONT);
    	}
    	*/
    	
    
        	
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
    	if(rightStick.getRawButton(3)) 
    	{
    		clawAssemblyCylinder.goOut();
        }
        else if(rightStick.getRawButton(2)) 
        {
        	clawAssemblyCylinder.goIn();
        }   	

    	
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
    		drive.setReversed(true);
    	else if (rightStick.getRawButton(11))
    		drive.setReversed(false);
    
    	if(rightStick.getRawButton(1) && shimmy1 == false)
    	{
    		shimmy1 = true;
    		driveShimmy.reset();
    		driveShimmy.start();
    	}

    	//done
    	// One sec
    	// can ou unmute your mic?
    		if (shimmy1 == true && driveShimmy.get() > 0.0 && shimmy2 == false)
    		{
    			drive.driveL(-0.5);
    			drive.driveR(0.5);
    			shimmy2 = true;
    		}
    		if (shimmy2 == true && driveShimmy.get() > 0.3 && shimmy3 == false)
    		{
    			drive.driveL(0.5);
    			drive.driveR(-0.5);
    			shimmy3 = true;
    		}
    		if (shimmy3 == true && driveShimmy.get() > 0.6 && shimmy4 == false)
    		{
    			drive.driveL(-0.5);
    			drive.driveR(-0.5);
    			shimmy4 = true;
    		}
    		if (shimmy4 == true && driveShimmy.get() > 1.1 && shimmyStop == false)
    		{
    			drive.driveL(0.0);
    			drive.driveR(0.0);
    			shimmyStop = true;
    			shimmy1 = false;
    			
    		}
    }
    }

