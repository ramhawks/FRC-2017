package org.usfirst.frc.team3090.robot;

public enum Path {
	TEST("Test",
			new DistanceBehind(1),
			new Rotation(90),
			new DistanceBehind(1)),
	LEFT("Left",
			new DistanceBehind(2.5),
			new Rotation(60),
			new DistanceAhead(0.4)),
	MIDDLE("Middle",
			new DistanceAhead(0.4)),
	RIGHT("Right",
/*			new DistanceBehind(2.5),
			new Rotation(-60),
			new DistanceAhead(0.4)),*/
			new ForwardSecond(Robot.getTimeForDistance(2.5)),
			new Rotation(-60),
			new ForwardSecond(Robot.getTimeForDistance(.5))),
	SIXTY("Sixty",
			new Rotation(60)),
	FIVESEC("5 sec",
			new ForwardSecond(5000));

	public final String name;

	public final Step[] steps;

	private Path(String name, Step... steps) {
		this.name = name;
		this.steps = steps;
	}

}
