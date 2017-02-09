package org.usfirst.frc.team3090.robot;

public enum Path {
	FIRST(
		new Distance(24),
		new Rotation(90),
		new Distance(24)
	);
	
	public Step[] steps;
	
	private Path(Step ... steps) {
		this.steps = steps;
	}
	
}
