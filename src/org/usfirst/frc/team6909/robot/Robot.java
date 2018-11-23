/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private DifferentialDrive m_robotDrive
			= new DifferentialDrive(new Spark(0), new Spark(1));
	private DifferentialDrive m_robotDrive2
	= new DifferentialDrive(new Spark(2), new Spark(3));
	private XboxController c =new XboxController(0);
	private Joystick m_stick = new Joystick(0);
//	private Timer m_timer = new Timer();\
	private PWMTalonSRX lift =new PWMTalonSRX(4);
	private PWMTalonSRX arm1 =new PWMTalonSRX(5);
	private PWMTalonSRX arm2 =new PWMTalonSRX(6);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
//		m_timer.reset();
//		m_timer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		/* if (m_timer.get() < 2.0) {
			m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
		} else {
			m_robotDrive.stopMotor(); // stop robot
		}
		/*if (m_timer.get() < 2.0) {
			m_robotDrive2.arcadeDrive(0.5, 0.0); // drive forwards half speed
		} else {
			m_robotDrive2.stopMotor(); // stop robot
		}*/
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		m_robotDrive.arcadeDrive(m_stick.getY(Hand.kLeft)/1.5, m_stick.getX(Hand.kLeft)/1.5);
		m_robotDrive2.arcadeDrive(-m_stick.getY(Hand.kLeft)/1.5, -m_stick.getX(Hand.kLeft)/1.5);
		if(c.getBButton()) {
			lift.set(1);
		}else {
			lift.set(0);
		}
		if(c.getYButton()) {
			arm1.set(0.3);
			arm2.set(-0.3);
		}else if(c.getXButton()) {
			arm1.set(-0.3);
			arm2.set(0.3);
		}else {
			arm1.set(0);
			arm2.set(0);
		}
	
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
