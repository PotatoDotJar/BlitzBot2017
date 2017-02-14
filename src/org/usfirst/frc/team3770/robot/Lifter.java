package org.usfirst.frc.team3770.robot;

import com.ctre.CANTalon;

public class Lifter {
	
	CANTalon rightMotor;
	CANTalon leftMotor;
	
	public Lifter(int rightMotorID, int leftMotorID) {
		rightMotor = new CANTalon(rightMotorID);
		leftMotor = new CANTalon(leftMotorID);
		
		
	}
	
	public void setLiftSpeed(double speed) {
		double converted = -speed;
		
		//if(converted < 0) {
			rightMotor.set(converted);
			leftMotor.set(converted * -1);
		//}
	}
	
	public double getRightCurrent() {
		return rightMotor.getOutputCurrent();
	}
	
	public double getLeftCurrent() {
		return leftMotor.getOutputCurrent();
	}
	
	
}
