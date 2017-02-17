package org.usfirst.frc.team3090.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class DistanceBehind extends Step {

	public final double meters;

	public DistanceBehind(double meters) {
		if (meters < 0.3) {			
			DriverStation.reportWarning("One of the autonomous steps is a distance fewer than 30 cm, which is too short", false);
		}
		
		this.meters = meters;
	}

}
