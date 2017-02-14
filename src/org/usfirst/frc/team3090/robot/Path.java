package org.usfirst.frc.team3090.robot;

public enum Path {
	TEST("First", new Distance(24), new Rotation(90), new Distance(24)),
	LEFT("Left"),
	MIDDLE("Middle",
			new Distance(93.25)),
	RIGHT("Right");

	public final String name;

	public final Step[] steps;

	private Path(String name, Step... steps) {
		this.name = name;
		this.steps = steps;
	}

}
