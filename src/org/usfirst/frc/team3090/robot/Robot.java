package org.usfirst.frc.team3090.robot;

import java.nio.ByteBuffer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot implements PIDOutput {
	public boolean debugging;

	RobotDrive myRobot;
	Joystick stick;

	Compressor compressor;

	private Path chosen_path;
	private boolean path = false;

	SendableChooser<String> chooser;

	AHRS ahrs;
	PIDController rotationController;

	private static final double kP = 0.03;
	private static final double kI = 0.00;
	private static final double kD = 0.00;
	protected static final double kF = 0.00;

	static final double kToleranceDegrees = 2.0f;

	double rotateToAngleRate;

	private boolean switching_gears;
	private long last_time;
	private DoubleSolenoid gear_switch;
	private boolean is_gear_fast;
	private boolean setpoint_changed;

	// Thread vision_thread;

	// Thread smart_dashboard_info;

	AnalogInput pressure_sensor;

	private AnalogInput distance_behind;
	private AnalogInput distance_ahead;

	@Override
	public void robotInit() {
		debugging = Preferences.getInstance().getBoolean("Debug", false);

		chooser = new SendableChooser<>();
		for (Path p : Path.values()) {
			chooser.addObject(p.name, p.name);
		}

		chooser.addDefault("Nicht", "Nicht");

		SmartDashboard.putData("Auto modes", chooser);

		Parts.init();

		myRobot = new RobotDrive(Parts.back_left, Parts.front_left, Parts.back_right, Parts.front_right);
		myRobot.setSafetyEnabled(false);
		stick = new Joystick(0);

		compressor = new Compressor();
		compressor.setClosedLoopControl(true);

		CameraServer.getInstance().startAutomaticCapture();

		pressure_sensor = new AnalogInput(1);

		putNumber("Lift", 0.5);

		myRobot.setExpiration(0.1);

		try {
			ahrs = new AHRS(I2C.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		rotationController = new PIDController(kP, kI, kD, ahrs, this);

		rotationController.setInputRange(-180.0f, 180.0f);
		rotationController.setOutputRange(-0.5, 0.5);
		rotationController.setAbsoluteTolerance(kToleranceDegrees);
		rotationController.setContinuous(true);

		LiveWindow.addActuator("DriveSystem", "Rotate Controller", rotationController);

		switching_gears = false;
		last_time = -1;
		gear_switch = new DoubleSolenoid(1, 0, 1);
		is_gear_fast = false;
		setpoint_changed = false;

		distance_behind = new AnalogInput(2);
		distance_ahead = new AnalogInput(3);

	}

	@Override
	public void autonomousInit() {
		// SmartDashboard.putData("Auto modes", chooser);

		String autoSelected = chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		for (Path p : Path.values()) {
			if (p.name.equals(autoSelected)) {
				chosen_path = p;
				path = true;
				break;
			}
		}

		index = 0;
		ahrs.reset();
		ahrs.resetDisplacement();
	}

	int index;

	@Override
	public void autonomousPeriodic() {
		values();

		if (path) {

			Step step = chosen_path.steps[index];

			if (step instanceof DistanceBehind) {

				DriverStation.reportWarning("Behind", false);

				DistanceBehind d = (DistanceBehind) step;

				if (getMetersBehind() >= d.meters) {

					myRobot.arcadeDrive(0, 0);

					index++;

					ahrs.reset();

				} else {

					myRobot.arcadeDrive(-0.5, 0);

				}

			} else if (step instanceof DistanceAhead) {
				DriverStation.reportWarning("Ahead", false);
				DistanceAhead d = (DistanceAhead) step;

				if (getMetersAhead() >= d.meters) {

					myRobot.arcadeDrive(0, 0);

					index++;

					ahrs.reset();

				} else {

					myRobot.arcadeDrive(-0.5, 0);

				}

			} else if (step instanceof Rotation) {
				DriverStation.reportWarning("Rotation", false);
				Rotation r = (Rotation) step;

				rotationController.setSetpoint(r.angle);
				rotationController.enable();

				if (r.init) {
					if (rotationController.get() != 0) {

						myRobot.arcadeDrive(0, rotateToAngleRate);

					} else {

						index++;

						ahrs.reset();

						myRobot.arcadeDrive(0, 0);

					}
				}

			}

			if (!step.init)
				step.init = true;

			if (index >= chosen_path.steps.length)
				path = false;

			DriverStation.reportWarning("Index: " + index, false);
			DriverStation.reportWarning("Step: " + step.init, false);
			DriverStation.reportWarning("Path: " + path, false);

		}

	}

	@Override
	public void disabledInit() {
		compressor.stop();
	}

	@Override
	public void teleopInit() {
		ahrs.reset();
		ahrs.resetDisplacement();
		rotationController.setSetpoint(0);
	}

	boolean b = false;

	@Override
	public void teleopPeriodic() {
		values();

		Parts.ballShooter.set(stick.getRawAxis(3));

		// double lift_speed = SmartDashboard.getNumber("Lift", 0.5);
		double lift_speed = Preferences.getInstance().getDouble("Lift", 0.5);

		if (stick.getRawButton(5)) {
			Parts.lift_1.set(lift_speed);
			Parts.lift_2.set(lift_speed);
		} else if (stick.getRawButton(6)) {
			Parts.lift_1.set(-lift_speed);
			Parts.lift_2.set(-lift_speed);
		} else {
			Parts.lift_1.set(0.0);
			Parts.lift_2.set(0.0);
		}

		// B toggles lift speed
		if (stick.getRawButton(2)) {
			if (!b) {
				if (lift_speed == 0.5)
					Preferences.getInstance().putDouble("Lift", 1);
				else
					Preferences.getInstance().putDouble("Lift", 0.5);
			}
			b = true;
		} else
			b = false;

		// Y toggles gears
		if (stick.getRawButton(4)) {
			if (is_gear_fast)
				is_gear_fast = false;
			else
				is_gear_fast = true;

			switching_gears = true;
		}

		if (switching_gears) {

			if (last_time < 0) {
				last_time = System.currentTimeMillis();
			} else {
				if (System.currentTimeMillis() - last_time >= 500) {
					switching_gears = false;
					last_time = -1;
				} else {

					if (is_gear_fast) {
						gear_switch.set(Value.kForward);
					} else {
						gear_switch.set(Value.kReverse);
					}

				}
			}

		} else {
			gear_switch.set(Value.kOff);
		}

		/*
		 * if (Math.abs(stick.getRawAxis(4)) < 0.1) { if (setpoint_changed) {
		 * setpoint_changed = false; ahrs.reset();
		 * rotationController.setSetpoint(0); }
		 * 
		 * rotationController.enable();
		 * 
		 * if (!switching_gears) myRobot.arcadeDrive(stick.getRawAxis(1),
		 * rotateToAngleRate);
		 * 
		 * } else { rotationController.disable();
		 * 
		 * if (!switching_gears) { myRobot.arcadeDrive(stick.getRawAxis(1),
		 * stick.getRawAxis(4)); setpoint_changed = true; } }
		 */

		myRobot.arcadeDrive(stick.getRawAxis(1), stick.getRawAxis(4));

		Timer.delay(0.0005);
	}

	public double getMetersBehind() {
		return (distance_behind.getVoltage() * 1024) / 1000;
	}

	public double getMetersAhead() {
		return (distance_ahead.getVoltage() * 1024) / 1000;
	}

	public void putNumber(String key, double value) {
		// if (debugging)
		// SmartDashboard.putNumber(key, value);
		Preferences.getInstance().putDouble(key, value);
	}

	public void putBoolean(String key, boolean value) {
		// if (debugging)
		// SmartDashboard.putBoolean(key, value);
		Preferences.getInstance().putBoolean(key, value);
	}

	public void putString(String key, String value) {
		// if (debugging)
		Preferences.getInstance().putString(key, value);
	}

	public void putByteArray(String key, byte[] value) {
		// if (debugging) {
		Preferences.getInstance().putInt(key, ByteBuffer.wrap(value).getInt());
		// }
	}

	public void values() {
		SmartDashboard.putNumber("PSI", (pressure_sensor.getVoltage() - 0.5) / 0.018);
		putNumber("Pressure Sensor Voltage", pressure_sensor.getVoltage());
		putNumber("Pressure Sensor Value", pressure_sensor.getValue());

		debugging = Preferences.getInstance().getBoolean("Debug", true);

		putNumber("axis.left.x", stick.getX());
		putNumber("axis.left.y", stick.getY());
		putBoolean("button.a", stick.getRawButton(1));
		putBoolean("button.b", stick.getRawButton(2));
		putBoolean("button.x", stick.getRawButton(3));
		putBoolean("button.y", stick.getRawButton(4));
		putBoolean("button.lb", stick.getRawButton(5));
		putBoolean("button.rb", stick.getRawButton(6));
		putBoolean("button.back", stick.getRawButton(7));
		putBoolean("button.start", stick.getRawButton(8));
		putBoolean("button.left_stick", stick.getRawButton(9));

		putNumber("ahrs.vY", ahrs.getVelocityY());

		putNumber("ahrs.dX", ahrs.getDisplacementX());
		putNumber("ahrs.dY", ahrs.getDisplacementY());
		putNumber("ahrs.dZ", ahrs.getDisplacementZ());
		putNumber("ahrs.angle", ahrs.getAngle());

		putNumber("d.voltage", distance_behind.getVoltage());
		putNumber("d.value", distance_behind.getValue());

		putString("distance behind", (distance_behind.getVoltage() * 1024) + "mm");
		putString("distance ahead", (distance_ahead.getVoltage() * 1024) + "mm");
	}

	@Override
	public void testInit() {
		compressor.start();
	}

	@Override
	public void testPeriodic() {
		values();
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}
