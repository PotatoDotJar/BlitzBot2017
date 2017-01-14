package org.usfirst.frc.team3770.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Debug {
	
	
	
	

    // A shorter print function for the DriveStation.
    public void print(int index, String line) {
    	clearDashboard();
    	SmartDashboard.putString("DB/String " + index, line);
    }
    // Clears the SmartDashboard
    public void clearDashboard() {
    	for(int i = 0; i <= 9; i++) {
    		SmartDashboard.putString("DB/String " + i, "");
    	}
    }
}
