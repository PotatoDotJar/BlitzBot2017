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

import javax.xml.ws.handler.MessageContext.Scope;

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
    
    private final int Left_Motor_1_ID		 = 
    private final int Left_Motor_2_ID		 = 
    private final int Right_Motor_1_ID		 = 
    private final int Right_Motor_2_ID		 = 
    
    private final int VISION_LED_RELAY_PORT  = 0;   

    private final int CLAW_CYLINDER_IN_PORT = 1;
    private final int CLAW_CYLINDER_OUT_PORT = 0;
    
    private final int CLAW_ROTATE_CYLINDER_IN_PORT = 3;
    private final int CLAW_ROTATE_CYLINDER_OUT_PORT = 2;
    
    private final int CLAW_ASSEMBLY_CYLINDER_IN_PORT = 5;
    private final int CLAW_ASSEMBLY_CYLINDER_OUT_PORT = 4;
    
    private final int TRACTION_WHEEL_CYLINDER_IN_PORT = 6;
    
    
    final int SWITCH_PORT = 0;
    
    // Declare objects for mechanical units
    DriveSystem drive;
    Joystick leftStick, rightStick;             // Joysticks
    
    Relay visionLedRelay;                       // Vision light switch
 
    ActuatorDouble clawCylinder;                // Claw Cylinder
    ActuatorDouble clawRotateCylinder;			// Claw Rotate Cylinder
    ActuatorDouble clawAssemblyCylinder;		// Claw Assembly Cylinder
    Solenoid tractionWheel;
    
    
    
    CameraSystem cameraSystem; 					// Manage cameras - front/back
    
    // Timer object(s)
    Timer autonClock;
    
    Debug debug;                                // Debug Utility Class
    
    // Declare utility variables
    boolean stoppedAtWall;
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
        
        // Pneumatics
        clawCylinder = new ActuatorDouble(CLAW_CYLINDER_IN_PORT, CLAW_CYLINDER_OUT_PORT, ActuatorStatus.IN);
        clawAssemblyCylinder = new ActuatorDouble(CLAW_ASSEMBLY_CYLINDER_IN_PORT, CLAW_ASSEMBLY_CYLINDER_OUT_PORT, ActuatorStatus.IN);
        clawRotateCylinder = new ActuatorDouble(CLAW_ROTATE_CYLINDER_IN_PORT, CLAW_ROTATE_CYLINDER_OUT_PORT, ActuatorStatus.IN);
        tractionWheel = new Solenoid(TRACTION_WHEEL_CYLINDER_IN_PORT);
        
        
        // Initializer various objects
        //sonar = new AnalogInput(SONAR_ANALOG_PORT);
        //IRSENSOR = new AnalogInput(ANALOG_IR_SENSOR_PORT);
        //cylinder = new ActuatorDouble(CYLINDER_IN_PORT, CYLINDER_OUT_PORT, ActuatorStatus.IN);
        //distanceTrigger = new DigitalInput(DIGITAL_DISTANC_SENSOR_PORT);
        
        
        
        visionLedRelay = new Relay(VISION_LED_RELAY_PORT, Direction.kForward);
        cameraSystem = new CameraSystem();
        
        
        tractionWheel.set(false);

        
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
    /*
    if(distanceTrigger.get()==false)
    {
    	stoppedAtWall = true;
    }
    	if (autonClock.get() < 2.0)	
    	{
    		leftMotor.set(0.5);
            rightMotor.set(0.5);
    	}
    	else
    	{
    		leftMotor.set(0.0);
            rightMotor.set(0.0);
    	}
    	
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
    	*/
}
    
    
    // =======================================================================
    public void teleopPeriodic() 
    {
        // right = IRSENSOR.getVoltage();
        
        // Set drive motors to current joy stick values
        drive.driveL(leftStick.getY());
        drive.driveR(rightStick.getY());        
        
        
    	
    	/*
        debug.print(1, "Sonar: " + sonar.getVoltage());
        debug.print(0, "IR Sensor: " + IRSENSOR.getVoltage());
        debug.print(2, "Digital Sensor: " + distanceTrigger.get());
        */
        
        
        // Manage any new control events
    	updateControls();
        
        clawCylinder.manageActions();
        clawRotateCylinder.manageActions();
        clawAssemblyCylinder.manageActions();
        
        cameraSystem.update();
        //debug.print(1, "PID: " + approachControl.get());
        
        visionLedRelay.set(Value.kOn);
        
    }
    
    // =======================================================================    
    public void updateControls()
    {
    	
    	
    	
    	if (rightStick.getRawButton(11)) {
    		cameraSystem.setCamera(Mode.BACK);
    	}
    	else if (rightStick.getRawButton(12)) {
    		cameraSystem.setCamera(Mode.FRONT);
    	}    	
    	
    }
}