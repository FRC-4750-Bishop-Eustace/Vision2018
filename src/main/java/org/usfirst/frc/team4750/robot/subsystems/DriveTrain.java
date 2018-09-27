/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4750.robot.subsystems;

import org.usfirst.frc.team4750.robot.commands.TankDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem implements PIDOutput {

	// Motors
	VictorSP frontLeftMotor;
	VictorSP frontRightMotor;
	VictorSP backLeftMotor;
	VictorSP backRightMotor;
	// Motor groups
	SpeedControllerGroup leftMotors;
	SpeedControllerGroup rightMotors;
	// Robot drive
	DifferentialDrive robotDrive;
	// Hold PID output
	double pidOutput;

	public DriveTrain(int FrontLeftMotorPort, int FrontRightMotorPort, int BackLeftMotorPort, int BackRightMotorPort) {
		// Initialize motors to motor ports
		frontLeftMotor = new VictorSP(FrontLeftMotorPort);
		frontRightMotor = new VictorSP(FrontRightMotorPort);
		backLeftMotor = new VictorSP(BackLeftMotorPort);
		backRightMotor = new VictorSP(BackRightMotorPort);
		// Initialize speed controller groups to each side
		leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
		rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
		// Initialize two side differential drive
		robotDrive = new DifferentialDrive(leftMotors, rightMotors);
	}

	public void initDefaultCommand() {
		// Call tank drive command when initialized
		setDefaultCommand(new TankDrive());
	}

	// Drive with joysticks (default command)
	public void joystickDrive(Joystick left, Joystick right) {
		// System.out.println("joystickDrive() is running!");
		robotDrive.tankDrive(-left.getY(), -right.getY());
		// System.out.println("DriveTrain.joystickDrive() running!");
	}

	// Set specific motor speeds
	public void setMotorSpeeds(double left, double right) {
		// Set motor speeds (inverted)
		robotDrive.tankDrive(-left, -right);
		// Output speeds
		SmartDashboard.putNumber("left speed", -left);
		SmartDashboard.putNumber("right speed", -right);
		// System.out.println("DriveTrain.setMotorSpeeds() running!");
	}

	@Override // Get PID output
	public void pidWrite(double output) {
		pidOutput = output;
		// SmartDashboard.putNumber("pidOutput", output);
	}

	// Opposite motor outputs for turning with PID
	public void pidTurn() {
		// Set motor speeds to PID output
		robotDrive.tankDrive(pidOutput, -pidOutput);
		// System.out.println("DriveTrain.pidTurn() running!");
	}

	// Drive with PID
	public void pidDrive() {
		// Set motor speeds to PID output
		robotDrive.tankDrive(pidOutput, pidOutput);
		// System.out.println("DriveTrain.pidDrive() running!");
	}
	
	// Set speeds for vision drive
	public void spin() {
		robotDrive.tankDrive(.6, -.6);
		System.out.println("spin() running!");
	}

	// Stop motors
	public void brake() {
		robotDrive.stopMotor();
		// System.out.println("DriveTrain.brake() running!");
	}
}
