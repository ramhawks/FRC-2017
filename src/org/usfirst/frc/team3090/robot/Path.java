package org.usfirst.frc.team3090.robot;

public enum Path {
/*	TEST("Test",
			new DistanceBehind(1),
			new Rotation(90),
			new DistanceBehind(1)),
	LEFT("Left",
			new DistanceBehind(2.5),
			new Rotation(60),
			new DistanceAhead(0.4)),
	MIDDLE("Middle",
			new DistanceAhead(0.4)),*/
	/*RIGHT("Right",
/*			new DistanceBehind(2.5),
			new Rotation(-60),
			new DistanceAhead(0.4)),
			new ForwardSecond(Robot.getTimeForDistance(2.5)),
			new Rotation(-60),
			new ForwardSecond(Robot.getTimeForDistance(.5))),*/
	SHOOT("Shoot 3 sec",
			new ShootSecond(3000)),
	THREESEC("Forward 3 sec",
			new ForwardSecond(3000)),
	BOILER_RIGHT("Boiler on Right",
			new ShootSecond(5000),
			new Rotation(-45),
			new BackwardSecond(3000)),
	BOILER_LEFT("Boiler on Left",
			new ForwardSecond(500),
			new ShootSecond(5000),
			new Rotation(45),
			new ForwardSecond(3000)),
	MIDDLE_PEG("Middle Peg",
			new ForwardSecond(4000)),
	NINETY("Rotate 90 Degrees",
			new Rotation(90));

	public final String name;

	public final Step[] steps;

	private Path(String name, Step... steps) {
		this.name = name;
		this.steps = steps;
	}

}
