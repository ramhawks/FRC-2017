package org.usfirst.frc.team3090.robot;

public enum Path {
	FIRST("First", 
			new Distance(24),
			new Rotation(90),
			new Distance(24));

	public final String name;

	public final Step[] steps;

	private Path(String name, Step... steps) {
		this.name = name;
		this.steps = steps;
	}

}
