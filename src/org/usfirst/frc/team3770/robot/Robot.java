// =======================================================================
// Team 3770 BlitzCreek - 2017 Robot Code
// Main Driver Module
// =======================================================================

package org.usfirst.frc.team3770.robot;

import org.usfirst.frc.team3770.robot.ActuatorDouble.ActuatorStatus;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.CANTalon;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

// =======================================================================
public class Robot extends IterativeRobot
{
    // Declare constant values
    private final int LEFT_STICK_USB_PORT    = 1;
    private final int RIGHT_STICK_USB_PORT   = 0;
    
    private final int RIGHT_CAN_MOTOR_ID   = 2;
    private final int LEFT_CAN_MOTOR_ID    = 3;
    private final int AUX_CAN_MOTOR_ID	 =   4;
    
    private final int SONAR_ANALOG_PORT   = 2; 
    final int VISION_LED_RELAY_PORT = 0;
    final int ANALOG_IR_SENSOR_PORT = 3;
    final int DIGITAL_DISTANC_SENSOR_PORT = 1;

    private final int CYLINDER_IN_PORT = 1;
    private final int CYLINDER_OUT_PORT = 0;
    
    final int SWITCH_PORT = 0;
    
    // Declare objects for mechanical units
    CANTalon leftMotor, rightMotor, auxMotor;   // Motors
    Joystick leftStick, rightStick;             // Joysticks
    AnalogInput sonar, IRSENSOR;                          
    DigitalInput distanceTrigger;
    Relay visionLedRelay;                       // Vision light switch
 
    ActuatorDouble cylinder;                    // Cylinder 1  
//    CameraSystemK cameraManager;                 // Manage cameras - front/back
    
    // Timer object(s)
    Timer autonClock;
    
    Debug debug;                                // Debug Utility Class
    
    // Declare utility variables
    double left,right, aux;
    boolean stoppedAtWall;
    // =======================================================================
    public void robotInit() 
    {
    	// This statement alone will send one USB camera image to the screen
        CameraServer.getInstance().startAutomaticCapture();
 	
        // Instantiate robot objects by calling constructors
        leftMotor  = new CANTalon(LEFT_CAN_MOTOR_ID);      
        rightMotor = new CANTalon(RIGHT_CAN_MOTOR_ID);
        auxMotor = new CANTalon(AUX_CAN_MOTOR_ID);
        
        // Create Debug object
        debug = new Debug();
        
        // Initialize the Joysticks
        leftStick  = new Joystick(LEFT_STICK_USB_PORT);     
        rightStick = new Joystick(RIGHT_STICK_USB_PORT);
        
        // Initializer various objects
        sonar = new AnalogInput(SONAR_ANALOG_PORT);
        IRSENSOR = new AnalogInput(ANALOG_IR_SENSOR_PORT);
        cylinder = new ActuatorDouble(CYLINDER_IN_PORT, CYLINDER_OUT_PORT, ActuatorStatus.IN);        
        visionLedRelay = new Relay(VISION_LED_RELAY_PORT, Direction.kForward);
        distanceTrigger = new DigitalInput(DIGITAL_DISTANC_SENSOR_PORT);
       // cameraManager =  new CameraSystemK();
        
        
       

        
        // Clear the dashboard
        debug.clearDashboard();
        System.out.println("=============ROBOT INITIALIZED!=============");
    }

    // =======================================================================    
    public void autonomousInit()
    {
    	autonClock = new Timer();
    	autonClock.reset();
    	autonClock.start();
    	stoppedAtWall = false;
    }
    
    // =======================================================================
    public void autonomousPeriodic()
    { 
    if(distanceTrigger.get()==false)
    {
    	stoppedAtWall = true;
    }
    /*	if (autonClock.get() < 2.0)	
    	{
    		leftMotor.set(0.5);
            rightMotor.set(0.5);
    	}
    	else
    	{
    		leftMotor.set(0.0);
            rightMotor.set(0.0);
    	}
    	*/
    	if(stoppedAtWall==false)
    	{
    
    	
    		 if(IRSENSOR.getVoltage()<=.4)
	    	{
	    		rightMotor.set(1);
	    		leftMotor.set(1);
	    	}
	
	    	else if (IRSENSOR.getVoltage()<=.6)
	    	{
	    		rightMotor.set(.75);
	    		leftMotor.set(.75);
	    	}
	
	    	else if (IRSENSOR.getVoltage()<=1)
	    	{
	    		rightMotor.set(.5);
	    		leftMotor.set(.5);
	    	}
    	}
    	else
    	{
    		rightMotor.set(0);
    		leftMotor.set(0);
    	}
}
    
    // =======================================================================
    public void teleopPeriodic() 
    {
    	// Get joy stick values (-1 ... 0 ... 1)
        left  = leftStick.getY();
        right = rightStick.getY();
       // right = IRSENSOR.getVoltage();
        
        // Set drive motors to current joy stick values
        leftMotor.set(left);
        rightMotor.set(right);        
        
        // Manage any new control events
    	updateControls();
        
        debug.print(1, "Sonar: " + sonar.getVoltage());
        debug.print(0, "IR Sensor: " + IRSENSOR.getVoltage());
        debug.print(2, "Digital Sensor: " + distanceTrigger.get());
        
        cylinder.manageActions();
        //debug.print(1, "PID: " + approachControl.get());
        
    }
    
    // =======================================================================    
    public void updateControls()
    {
    	// Bring cylinder in
    	if(leftStick.getRawButton(3)) 
    	{
    		if(cylinder.getStatus() == ActuatorStatus.IN) 
    		{
    			cylinder.goOut();
    		}
    	}
    	else if(leftStick.getRawButton(4)) 
    	{
    		if(cylinder.getStatus() == ActuatorStatus.OUT) 
    		{
    			cylinder.goIn();
    		}
    	}
    	else if (leftStick.getRawButton(11)) {
    		visionLedRelay.set(Value.kOn);
    	}
    	else if (leftStick.getRawButton(10)) {
    		visionLedRelay.set(Value.kOff);
    	}
    	
    
    	/*
    	else if (rightStick.getRawButton(11)) {
    		cameraManager.setTarget(1);
    	}
    	else if (rightStick.getRawButton(12)) {
    		cameraManager.setTarget(0);
    	}    	
    	 */
    	
    	  	
    	/*
    	else if(rightStick.getRawButton(9)) {
    		auxMotor.set(1.0);
    	}
    	else if(rightStick.getRawButton(10)) {
    		auxMotor.set(0.0);
    	}
    	else if(theSwitch.get() == false){
    		auxMotor.set(0.0);
    	}
    	*/
    }
}