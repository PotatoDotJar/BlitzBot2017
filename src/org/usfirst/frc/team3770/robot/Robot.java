// Team 3770 Robot
// PID testing

package org.usfirst.frc.team3770.robot;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

import org.usfirst.frc.team3770.robot.ActuatorDouble.ActuatorStatus;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{
    // Declare constant values
    private final int LEFT_STICK_USB_PORT    = 1;
    private final int RIGHT_STICK_USB_PORT   = 0;
    
    private final int LEFT_MOTOR_PORT    = 0;
    private final int RIGHT_MOTOR_PORT   = 1;
    private final int AUX_MOTOR_PORT	 = 2;
    
    private final int POT_ANALOG_PORT   = 0;
    
    private final int CYLINDER_IN_PORT = 1;
    private final int CYLINDER_OUT_PORT = 0;
    
    final int VISION_LED_RELAY_PORT = 0;
    
    // Declare objects
    Talon leftMotor, rightMotor, auxMotor;
    
    Joystick leftStick, rightStick;
    AnalogInput pot;
    
    // Dubug Utility Class
    Debug debug;
    
    // Declare utility variables
    double left,right, aux;
    
    // Add the pneumatics compressor
    Compressor compressor;
    
    // Add Pneumatic Cylinder
    ActuatorDouble cylinder;
    
    // Led relay for vision
    Relay visionLedRelay;
    
    /*
    PIDController approachControl;
    final double Kp = 0.2;
    final double Ki = 0.2;
    final double Kd = 0.2;
    */
    
    //-------------------------------------------------
    public void robotInit() 
    {
        // Instantiate robot objects by calling constructors
        leftMotor  = new Talon(LEFT_MOTOR_PORT);      
        rightMotor = new Talon(RIGHT_MOTOR_PORT);
        auxMotor = new Talon(AUX_MOTOR_PORT);
        
        // Create Debug object
        debug = new Debug();
        
        // leftMotor.setInverted(true);
        // leftMotor.setInverted(true);
        
        // Initialize the Joysticks
        leftStick  = new Joystick(LEFT_STICK_USB_PORT);     
        rightStick = new Joystick(RIGHT_STICK_USB_PORT);
        
        
        pot = new AnalogInput(POT_ANALOG_PORT);
        
        // Initialize compressor and start it
        compressor = new Compressor();
        compressor.start();
        
        cylinder = new ActuatorDouble(CYLINDER_IN_PORT, CYLINDER_OUT_PORT, ActuatorStatus.IN);
        
        
        visionLedRelay = new Relay(VISION_LED_RELAY_PORT, Direction.kForward);
        /*
        driveOutput = new DrivePIDoutput(leftMotor,rightMotor);
        pidSenseInput = new SonarPIDinput(pot);
        
        approachControl = new PIDController( Kp,  Ki,  Kd, pidSenseInput, driveOutput);
        approachControl.setInputRange(0.0, 40.0);
        approachControl.setOutputRange(0, 1.0);     
        
        approachControl.enable();
        */
        
        // Clear the dashboard
        debug.clearDashboard();
        System.out.println("=============ROBOT INITIALIZED!=============");
    }
    
    //-------------------------------------------------
    public void teleopPeriodic() 
    {
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
    	else if(leftStick.getRawButton(11)) {
    		visionLedRelay.set(Value.kOn);
    	}
    	else if(leftStick.getRawButton(12)) {
    		visionLedRelay.set(Value.kOff);
    	}
    	
        // Get joy stick values (-1 ... 0 ... 1)
        left  = leftStick.getY();
        right = rightStick.getY();
        aux = (pot.getVoltage()/5);
        
        
        // Set drive motors to current joy stick values
        leftMotor.set(left);
        rightMotor.set(right);
        auxMotor.set(aux);
        
        debug.print(0, "Pot: " + pot.getVoltage());
        //debug.print(1, "PID: " + approachControl.get());
        
    }
}