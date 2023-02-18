// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  public TankDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // called in every periodic
    double leftStickY = Robot.m_oi.getRawAxis(RobotMap.LEFT_STICK_Y);
    double rightStickY = Robot.m_oi.getRawAxis(RobotMap.RIGHT_STICK_Y);

    double triggerVal = Robot.m_oi.getRightTriggerAxis() - Robot.m_oi.getLeftTriggerAxis();
    double stick = Robot.m_oi.getLeftX() * RobotMap.TURNING_RATE;

    // this is using manning robotic's GTA drive where they control robot like the GTA game

    Robot.driveTrain.setLeftSpeed(triggerVal + stick);
    Robot.driveTrain.setRightSpeed(triggerVal - stick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
