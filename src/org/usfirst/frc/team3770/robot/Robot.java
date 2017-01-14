// Team 3770 Robot
// PID testing

package org.usfirst.frc.team3770.robot;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{
    // Declare constant values
    private final int LEFT_STICK_USB_PORT    = 1;
    private final int RIGHT_STICK_USB_PORT   = 0;
    
    private final int LEFT_MOTOR_PORT    = 0;
    private final int RIGHT_MOTOR_PORT   = 2;
    
    private final int SONAR_ANALOG_PORT   = 0;
    
    // Declare objects
    Talon leftMotor, rightMotor;
    Joystick leftStick, rightStick;
    AnalogInput sonar;
    
    DrivePIDoutput driveOutput;
    SonarPIDinput sonarInput;
    
    // Declare utility variables
    double left,right;
    
    PIDController approachControl;
    final double Kp = 0.2;
    final double Ki = 0.2;
    final double Kd = 0.2;
    
    //-------------------------------------------------
    public void robotInit() 
    {
        // Instantiate robot objects by calling constructors
        leftMotor  = new Talon(LEFT_MOTOR_PORT);      
        rightMotor = new Talon(RIGHT_MOTOR_PORT);
        
       // leftMotor.setInverted(true);
       // leftMotor.setInverted(true);
        
        leftStick  = new Joystick(LEFT_STICK_USB_PORT);     
        rightStick = new Joystick(RIGHT_STICK_USB_PORT);
        
        sonar = new AnalogInput(SONAR_ANALOG_PORT);
        
        driveOutput = new DrivePIDoutput(leftMotor,rightMotor);
        sonarInput = new SonarPIDinput(sonar);
        
        approachControl = new PIDController( Kp,  Ki,  Kd, sonarInput, driveOutput);
        approachControl.setInputRange(0.0, 40.0);
        approachControl.setOutputRange(0, 1.0);     
        approachControl.setSetpoint(15.0) ; 
        approachControl.enable();
    }
    
    //-------------------------------------------------
    public void teleopPeriodic() 
    {
        // Get joy stick values (-1 ... 0 ... 1)
        left  = leftStick.getY();
        right = rightStick.getY();
        
        // Set drive motors to current joy stick values
        leftMotor.set(left);
        rightMotor.set(right);
        
        
        SmartDashboard.putString("DB/String 0", "PID Source: " + sonarInput.pidGet());
        SmartDashboard.putString("DB/String 1", "PID Output: " + approachControl.get());
    }
}