package org.usfirst.frc.team3090.robot;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot implements PIDOutput {
	public static boolean debugging;

	RobotDrive myRobot;
	Joystick stick;

	private Path chosen_path;
	private boolean path = false;

	SendableChooser<String> chooser;

	AHRS ahrs;
	PIDController pidController;

	private static final double kP = 0.03;
	private static final double kI = 0.00;
	private static final double kD = 0.00;
	protected static final double kF = 0.00;

	static final double kToleranceDegrees = 2.0f;

	double rotateToAngleRate;

	/*
	 * AnalogInput sonar; AnalogInput sonarAlso;
	 */

	private boolean switching_gears;
	private long last_time;
	private DoubleSolenoid gear_switch;
	private boolean is_gear_fast;

	volatile Thread vision_thread;

	@Override
	public void robotInit() {
		debugging = false;

		Parts.init();

		myRobot = new RobotDrive(Parts.back_left, Parts.front_left, Parts.back_right, Parts.front_right);
		stick = new Joystick(0);

		vision_thread = new Thread(() -> {
			// Get the Axis camera from CameraServer
			AxisCamera camera = CameraServer.getInstance().addAxisCamera("axis-camera.local");
			camera.setResolution(640, 480);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Camera", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				/*
				 * Imgproc.rectangle(mat, new Point(100, 100), new Point(400,
				 * 400), new Scalar(255, 255, 255), 5);
				 */
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});

		vision_thread.start();

		chooser = new SendableChooser<>();
		for (Path p : Path.values()) {
			chooser.addObject(p.name, p.name);
		}

		SmartDashboard.putData("Auto modes", chooser);

		/*
		 * sonar = new AnalogInput(5); sonarAlso = new AnalogInput(0);
		 */

		putNumber("Lift", 0.5);

		myRobot.setExpiration(0.1);

		try {
			ahrs = new AHRS(I2C.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		pidController = new PIDController(kP, kI, kD, ahrs, this);

		pidController.setInputRange(-180.0f, 180.0f);
		pidController.setOutputRange(-1.0, 1.0);
		pidController.setAbsoluteTolerance(kToleranceDegrees);
		pidController.setContinuous(true);

		LiveWindow.addActuator("DriveSystem", "Rotate Controller", pidController);

		switching_gears = false;
		last_time = -1;
		gear_switch = new DoubleSolenoid(1, 0, 1);
		is_gear_fast = false;
	}

	@Override
	public void autonomousInit() {
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
	}

	int index;

	@Override
	public void autonomousPeriodic() {

		if (path) {

			Step step = chosen_path.steps[index];

			if (step instanceof Distance) {

				Distance d = (Distance) step;

				if (-ahrs.getDisplacementY() >= d.inches) {

					myRobot.drive(0, 0);

					index++;

					ahrs.reset();

				} else {

					myRobot.drive(1, 0);

				}

			} else if (step instanceof Rotation) {

				Rotation r = (Rotation) step;

				pidController.setSetpoint(r.angle);
				pidController.enable();

				if (r.init)
					if (pidController.get() != 0) {

						myRobot.drive(0, rotateToAngleRate);

					} else {

						index++;

						ahrs.reset();

						myRobot.drive(0, 0);

					}

			}

			if (!step.init)
				step.init = true;

			if (index >= chosen_path.steps.length)
				path = false;
		}

	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {

		double lift_speed = SmartDashboard.getNumber("Lift", 0.5);

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

		if (!switching_gears)
			myRobot.arcadeDrive(stick.getRawAxis(1), stick.getRawAxis(4));

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

		/*
		 * putNumber("Value", sonar.getValue()); putNumber("Voltage",
		 * sonar.getVoltage()); putNumber("Average Value",
		 * sonar.getAverageValue()); putNumber("Average Voltage",
		 * sonar.getAverageVoltage());
		 * 
		 * putNumber("ValueA", sonarAlso.getValue()); putNumber("VoltageA",
		 * sonarAlso.getVoltage()); putNumber("Average ValueA",
		 * sonarAlso.getAverageValue()); putNumber("Average VoltageA",
		 * sonarAlso.getAverageVoltage());
		 * 
		 * putNumber("Value Inches", sonar.getValue() / 0.0098);
		 * putNumber("Voltage Inches", sonar.getVoltage() / 0.0098);
		 * putNumber("Average Value Inches", sonar.getAverageValue() / 0.0098);
		 * putNumber("Average Voltage Inches", sonar.getAverageVoltage() /
		 * 0.0098);
		 * 
		 * putNumber("ValueA mm", sonarAlso.getValue() / 0.000977);
		 * putNumber("VoltageA mm", sonarAlso.getVoltage() / 0.000977);
		 * putNumber("Average ValueA mm", sonarAlso.getAverageValue() /
		 * 0.000977); putNumber("Average VoltageA mm",
		 * sonarAlso.getAverageVoltage() / 0.000977);
		 */

		Timer.delay(0.005);
	}

	public static void putNumber(String key, double value) {
		if (debugging)
			SmartDashboard.putNumber(key, value);
	}

	public static void putBoolean(String key, boolean value) {
		if (debugging)
			SmartDashboard.putBoolean(key, value);
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}
