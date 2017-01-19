package org.usfirst.frc.team3090.robot;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
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
	RobotDrive myRobot = new RobotDrive(0, 1);
	Joystick stick = new Joystick(0);
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();
	
	AHRS ahrs;
	PIDController pidController;
	
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	
	static final double kToleranceDegrees = 2.0f;
	
	Thread vision_thread;

	public Robot() {
		myRobot.setExpiration(0.1);
		
		try {
			ahrs = new AHRS(I2C.Port.kMXP);
		} catch (RuntimeException ex) {
			// TODO Auto-generated catch block
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		pidController = new PIDController(kP, kI, kD, ahrs, this);
		
		pidController.setInputRange(-180.0f,  180.0f);
	    pidController.setOutputRange(-1.0, 1.0);
	    pidController.setAbsoluteTolerance(kToleranceDegrees);
	    pidController.setContinuous(true);
	    
	    LiveWindow.addActuator("DriveSystem", "Rotate Controller", pidController);
	}

	@Override
	public void robotInit() {
		vision_thread = new Thread(() -> {
			// Get the Axis camera from CameraServer
			AxisCamera camera = CameraServer.getInstance().addAxisCamera("axis-camera.local");
			// Set the resolution
			camera.setResolution(640, 480);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("10.30.90.89", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				/*Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
						new Scalar(255, 255, 255), 5);*/
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		vision_thread.setDaemon(true);
		vision_thread.start();
		
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
	}

	@Override
	public void autonomousInit() {
		String autoSelected = chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		switch (autoSelected) {
		case customAuto:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 1.0); // spin at half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		case defaultAuto:
		default:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		}
	}	

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
	}
	
	@Override
	public void teleopPeriodic() {
		myRobot.setSafetyEnabled(true);

		myRobot.arcadeDrive(stick); // drive with arcade style (use right
										// stick)
		Timer.delay(0.005); // wait for a motor update time

	}
	
	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void pidWrite(double output) {
		
	}
}
