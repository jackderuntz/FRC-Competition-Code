package org.usfirst.frc.team4206.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4206.robot.subsystems.Vision;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

/**
 * 2017 Team 4206 Java Code
 */

public class Robot extends SampleRobot implements PIDOutput {
	//Controller
    Joystick driver, select;
    
    //Power Distribution Panel
    PowerDistributionPanel pdp;
    
    //Gyroscope
    AHRS ahrs;

    //Motor controllers
    CANTalon frontLeft;
    CANTalon rearLeft;
    CANTalon frontRight;
    CANTalon rearRight;
    CANTalon climbermaster;
    CANTalon climberslave;
    CANTalon gearLiftMaster;
    CANTalon gearLiftSlave;
    CANTalon gearWheel;
    
    //Drive train
    RobotDrive robotDrive;
    
    //I/O and vision
    //Vision vision;
    Preferences prefs;
    double autoDist;
    static final int feeder = 9, left = 5, center = 3, right = 7;
    
    //Climber power channels
    static final int slaveChannel = 1;
    static final int masterChannel = 2;
    
    //PID Configuration
    PIDController turnController;
    double rotateToAngleRate;
    static final double kP = .01;
    static final double kI = 0;
    static final double kD = 0;
    static final double kF = 0;
    static final double kToleranceDegrees = 1.0f;
    
    //Motor variables
    double move;
    double climbAccum;
    
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

    public Robot() {
    	camera.setResolution(360, 240);
    	camera.setFPS(20);
    	
    	//Power Distribution Panel
    	pdp = new PowerDistributionPanel();
    	
    	//Talon SRX Configuration
    	frontLeft = new CANTalon(1);
        rearLeft = new CANTalon(7);
        frontRight = new CANTalon(2);
        rearRight = new CANTalon(8);
        climbermaster = new CANTalon(4);
        climberslave = new CANTalon(3);
        gearLiftMaster = new CANTalon(5);
        gearLiftSlave = new CANTalon(6);
        gearWheel = new CANTalon(9);

        //PID and Sensory Devices
    	ahrs = new AHRS(Port.kMXP);
    	turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    	turnController.setSetpoint(0);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-.8, .8);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.enable();
        
        //I/O and Vision
        prefs = Preferences.getInstance();
        autoDist = prefs.getDouble("AutoPos", 15000);
        //vision = new Vision();
       
    	//Mecanum Drive Train
        robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	robotDrive.setInvertedMotor(MotorType.kFrontRight, true);	// invert the right side motors
    	robotDrive.setInvertedMotor(MotorType.kRearRight, true);		// you may need to change or remove this to match your robot
        robotDrive.setExpiration(0.1);
        robotDrive.setSafetyEnabled(false);
        
        //Encoders
        frontLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        frontRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rearLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rearLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        gearLiftSlave.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        
        //Climber
        climberslave.changeControlMode(CANTalon.TalonControlMode.Follower);
        climberslave.set(climbermaster.getDeviceID());
        climbAccum = 0.0;

        //Active gear
        gearLiftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        gearLiftSlave.set(gearLiftMaster.getDeviceID());
        gearLiftSlave.configEncoderCodesPerRev(360);
        
        //Controller
        driver = new Joystick(0);
        select = new Joystick(1);
        // feed 0
        // left 2
        // center 4
        // right 6
    }

    public void autonomous() {
    	// Mode defaults to false (center) in the unlikely case a digital input fails to initialize or is not polled
    	ahrs.reset();
    	int mode = 0;
    	boolean chute = select.getRawButton(feeder);
    	if (select.getRawButton(left) & select.getRawButton(right) | select.getRawButton(center)) mode = 0;
    	else if (select.getRawButton(left)) mode = 1;
    	else if (select.getRawButton(right)) mode = 2;
    	if (mode == 1 | mode == 2) {
    		double angle = 0, endPos = autoDist;
    		boolean gyroFirst = true;
    		double gyroOffset = 0, gyroStart = ahrs.getAngle();
    			if (mode == 1) {
    				endPos = autoDist; //19570 left chute longer
    				angle = 60;
    			}
    			else {
    				endPos = autoDist; //right boiler shorter
    				angle = -60;
    			}
    		
    		// rl is the relative position of the rear left encoder to when the robot starts the autonomous process
    		// _rl is the position of the rear left encoder, but may not be zero at the start: used to offset rl
    		// endPos is where the turn should occur to face the peg
    		// pegPos is not fully implemented, but is supposed to stop the robot from moving forward after it reaches the target.
    		//       may be replaced with a current check if the motor stalls when it hits the wall, indicating it has reached the target
    		// time is used to accurately determine when to begin driving towards the peg after the turn without a timer and multi-threading
    		double rl, _rl, time=0, safe_t=System.currentTimeMillis()+1000;
			 // we reset the gyroscope to ensure it is zero'd for the beginning of autonomous
			boolean flag = false; // flag used to run a set of commands once in a loop before changing states to prevent being run again
			_rl = -frontRight.getEncPosition(); // _rl used to offset rl
			turnController.enable(); // PID controller for turning is enabled, this was done earlier but is repeated just to be safe
			while (isAutonomous() & isEnabled() & System.currentTimeMillis() < safe_t+16000) {
				//SmartDashboard.putNumber("Front Right Position", frontRight.getEncPosition());
				//double xPos = vision.testPixy1();
				//rl = rearLeft.getEncPosition() - _rl;
				rl = (_rl + frontRight.getEncPosition()); // gets difference between where the encoder started and where it is currently, giving relative position
				//System.out.println(rl);
				double pidOut = 0.0075 * (rl-endPos); // simple P controller for determining power as we approach the target
				// conditional limits max power to 20%
				if (pidOut > 0.2) pidOut = 0.2;
				else if (pidOut < -0.2) pidOut = -0.2;
				// this breaks out of the loop if our encoder isn't returning values after a second, telling us it is not working.
				if (safe_t <= System.currentTimeMillis() & rl < 100) break;
				
				// conditional determines if we have reached the destination and applies proper commands
				if (rl <= endPos) robotDrive.mecanumDrive_Cartesian(0, pidOut, rotateToAngleRate, 0); // drive straight to distance using P and PID controllers
				else {
					// runs once: 
					// -sets end position to much further so it doesnt try to go forward again if rl somehow drops below endpos
					// -sets power to zero at the target
					// -sets turn controller to target angle
					// -waits 0.75 seconds
					// -sets time variable for turning without driving forward
					if (!flag) {
						flag = true;
						robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
						turnController.setSetpoint(angle);
						endPos = 10000000;
						Timer.delay(0.75);
						_rl-= -frontRight.getEncPosition();
						time = System.currentTimeMillis()+1500;
					}
					// conditional turns only for a second after the target is reached, then drives forward only if
					// the robot is on target with the peg within a tolerance of 0.25 volts, which is the analog output
					// of the PixyCam (vision)
					
					if (System.currentTimeMillis() >= time) robotDrive.mecanumDrive_Cartesian(0, -0.08, rotateToAngleRate, 0);
				}
				Timer.delay(0.01);
			}
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			turnController.setSetpoint(0); // sets the turn controller back to zero for safety
    	} else if (mode == 0) {
    		// drives straight for center peg or (for some reason) breaching base line
        	long time = System.currentTimeMillis()+5500; // schedules time for when to stop driving forward
        	ahrs.reset(); // resets the gyroscope for use
        	
        	while (isAutonomous() & isEnabled()) {
        		
        		if (System.currentTimeMillis() >= time - 4500) robotDrive.mecanumDrive_Cartesian(0, -.2, rotateToAngleRate, 0); // drives forward using PID for accuracy
        		/*
        		if (System.currentTimeMillis() > time - 2500 & !back) {
        			gearWheel.set(-0.175);
        			gearLiftMaster.set(.1);
        			robotDrive.mecanumDrive_Cartesian(0, -.35, rotateToAngleRate, 0);
        			if (System.currentTimeMillis() < time - 1250) back = true;
        		}
        		*/
        		if (System.currentTimeMillis() > time) break; // breaks out of loop if the current time has passed the scheduled time
        		Timer.delay(0.01);
        	}
        	robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0); // sets power to zero for safety
    	}
    }

/*    
    public void autonomous() {
    	
    	//Command cmd = new Left();
    	//cmd.start();
    	boolean flag = true;
    	long time = System.currentTimeMillis()+4500;
        //static double kToleranceDegrees = 1.0f;
    	ahrs.reset();
    	
    	while (isAutonomous() & isEnabled() & flag) {
    		//Scheduler.getInstance().run();   
    		//tick++;
    		robotDrive.mecanumDrive_Cartesian(0, -0.2, rotateToAngleRate, 0);
    		if (System.currentTimeMillis() > time) flag = false;
    		Timer.delay(0.01);
    	}
    	
    	robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	
    	 
    }
*/
    /**
     * Runs the motors with Mecanum drive.
     */
    public void operatorControl() {
    	boolean reverse = false, toggle = true;
        while (isOperatorControl() & isEnabled()) {
/*----------Active Gear---------------------------------------------------------*/
        	gearLiftMaster.set((-driver.getRawAxis(2) + driver.getRawAxis(3)) * 0.2);
        	
        	if (driver.getRawButton(5)) gearWheel.set(1);
    		else if (driver.getRawButton(6)) gearWheel.set(-.75);
    		else gearWheel.set(0);
        	
        	if (pdp.getCurrent(11) >= 9) {
        		driver.setRumble(RumbleType.kLeftRumble, 1);
        		driver.setRumble(RumbleType.kRightRumble, 1);
        	} else {
        		driver.setRumble(RumbleType.kLeftRumble, 0);
        		driver.setRumble(RumbleType.kRightRumble, 0);
        	}
        	
/*----------driver Dead Zone----------------------------------------------------*/
        	double y = 0;
        	double x = 0;
        	double turn = 0;
        	
        	if (driver.getRawButton(7) & toggle) {
        		reverse = !reverse;
        		toggle = false;
        	} else if (!driver.getRawButton(7)) toggle = true;
        	else toggle = false;
        	
        	if (!reverse) {
            	y = driver.getY();
            	x = driver.getX();
            	turn = driver.getRawAxis(4);
        	} else {
        		y = -driver.getY();
            	x = -driver.getX();
            	turn = driver.getRawAxis(4);
        	}
        	
/*----------Drive Train-------------------------------------------------------------*/        	
        	
            try {
            	robotDrive.mecanumDrive_Cartesian(x, y, turn, 0);
            } catch( RuntimeException ex ) {
                DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
            }
				
/*----------Climber-----------------------------------------------------------------*/
			if (driver.getRawButton(1)) climbermaster.set(-0.8);
			else climbermaster.set(0);
            
            SmartDashboard.putNumber("Climber Master Current", pdp.getCurrent(masterChannel));
            SmartDashboard.putNumber("Climber Slave Current", pdp.getCurrent(slaveChannel));
            

            

/*----------Encoders----------------------------------------------------------------*/
            double EncRearLeftPos = rearLeft.getEncPosition();
            double EncRearLeftSpeed = rearLeft.getSpeed();
            double FeetPerSecond = (((EncRearLeftSpeed/4096)*Math.PI*2)*3);
            SmartDashboard.putNumber("Rear Left Position", EncRearLeftPos);
            SmartDashboard.putNumber("Rear Right Position", rearRight.getEncPosition());
            SmartDashboard.putNumber("Front Left Position", frontLeft.getEncPosition());
            SmartDashboard.putNumber("Front Right Position", frontRight.getEncPosition());
            SmartDashboard.putNumber("Velocity Rear Left", FeetPerSecond);
            
/*----------Gyro--------------------------------------------------------------------*/
           
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
    }

	public void disabled() {
		rotateToAngleRate = 0;
		robotDrive.mecanumDrive_Cartesian(0,0,0,0);
		climbermaster.set(0);
		gearLiftMaster.set(0);
		gearWheel.set(0);
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;	
	}
}