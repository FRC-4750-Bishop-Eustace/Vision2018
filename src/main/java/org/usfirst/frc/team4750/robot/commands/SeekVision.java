/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4750.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team4750.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class SeekVision extends Command {

	// Is a target in view
	boolean hasTarget;
	// Speed to turn at
	double speed = 0.35;
	// Reports if the target is in view
	boolean isFinished = false;
	// Aim command
	Command aim;

	public SeekVision() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.vision);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		aim = new AimVision();
		requires(Robot.vision);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Get if the target is in view
		hasTarget = Robot.vision.getHasTarget();

		if (!hasTarget) { // If the target is not in view, set the motors to turn the robot
			Robot.driveTrain.setMotorSpeeds(speed, -speed);
		} else { // If the target is in view, finish the command
			aim.start();
			isFinished = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("SeekVision(): Target found!");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
