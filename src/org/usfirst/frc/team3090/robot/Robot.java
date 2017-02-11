package org.usfirst.frc.team3090.robot;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.AnalogInput;
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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot implements PIDOutput {
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

	AnalogInput sonar;
	AnalogInput sonarAlso;

	Solenoid ourTestSolenoid;

	public boolean soleState;

	volatile Thread vision_thread;

	DoubleSolenoid gear_switch;

	@Override
	public void robotInit() {
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

		sonar = new AnalogInput(5);
		sonarAlso = new AnalogInput(0);

		gear_switch = new DoubleSolenoid(1, 0, 1);

		SmartDashboard.putNumber("Lift", 0.5);

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

		ourTestSolenoid = new Solenoid(1);
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

		if (stick.getRawButton(1)) {
			gear_switch.set(Value.kForward);
		} else if (stick.getRawButton(4)) {
			gear_switch.set(Value.kReverse);
		} else if (stick.getRawButton(3)) {
			gear_switch.set(Value.kOff);
		}

		myRobot.arcadeDrive(stick.getRawAxis(1), stick.getRawAxis(4));

		SmartDashboard.putNumber("Value", sonar.getValue());
		SmartDashboard.putNumber("Voltage", sonar.getVoltage());
		SmartDashboard.putNumber("Average Value", sonar.getAverageValue());
		SmartDashboard.putNumber("Average Voltage", sonar.getAverageVoltage());

		SmartDashboard.putNumber("ValueA", sonarAlso.getValue());
		SmartDashboard.putNumber("VoltageA", sonarAlso.getVoltage());
		SmartDashboard.putNumber("Average ValueA", sonarAlso.getAverageValue());
		SmartDashboard.putNumber("Average VoltageA", sonarAlso.getAverageVoltage());

		SmartDashboard.putNumber("Value Inches", sonar.getValue() / 0.0098);
		SmartDashboard.putNumber("Voltage Inches", sonar.getVoltage() / 0.0098);
		SmartDashboard.putNumber("Average Value Inches", sonar.getAverageValue() / 0.0098);
		SmartDashboard.putNumber("Average Voltage Inches", sonar.getAverageVoltage() / 0.0098);

		SmartDashboard.putNumber("ValueA mm", sonarAlso.getValue() / 0.000977);
		SmartDashboard.putNumber("VoltageA mm", sonarAlso.getVoltage() / 0.000977);
		SmartDashboard.putNumber("Average ValueA mm", sonarAlso.getAverageValue() / 0.000977);
		SmartDashboard.putNumber("Average VoltageA mm", sonarAlso.getAverageVoltage() / 0.000977);

		if (stick.getRawButton(4) == true) {
			soleState = true;
		}
		if (stick.getRawButton(1) == true) {
			soleState = false;
		}
		ourTestSolenoid.set(soleState);

		Timer.delay(0.005);
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
