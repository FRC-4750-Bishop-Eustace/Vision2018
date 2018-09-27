/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4750.robot.commands;

import org.usfirst.frc.team4750.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command. You can replace me with your own command.
 */
public class AimVision extends Command {

	// Angle from center on x-axis
	double xOffset;
	// Angle from center on x-axis (corrected sign)
	float headingError;
	// Reports if the target is being aligned
	boolean isFinished = false;
	// Turn command
	Command turn;
	// Seek command
	Command seek;
	// If the target is in view
	boolean hasTarget;

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		seek = new SeekVision();
		System.out.println("AimVision() initialize"); 
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Get current offset on the x-axis
		xOffset = Robot.vision.getXOffset();
		SmartDashboard.putNumber("AimVision().headingError", headingError);
		// Get if the target is in view
		hasTarget = Robot.vision.getHasTarget();
		
		if(hasTarget) {
			System.out.println("AimVision() hasTarget"); 
			turn = new ControlledTurn((float) xOffset);
			turn.start();
			isFinished = true;
		}else {
			System.out.println("AimVision() does not have target"); 
			seek.start();
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
		System.out.println("AimVision(): stopped!");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}