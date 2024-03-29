// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automatic;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoMove extends CommandBase {
  final DriveTrain driveTrain;
  double targetPosition;
  
  /** Creates a new AutoBalance. */
  public AutoMove(DriveTrain driveTrain) {
    driveTrain.leftController.setP(0.0003);
    driveTrain.leftController.setI(0);
    driveTrain.leftController.setD(0);
    driveTrain.leftController.setFF(0);
    
    this.driveTrain = driveTrain;


    driveTrain.leftController.setReference(targetPosition, ControlType.kPosition);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void resetEncoder() {
    System.out.println("reset encoder");
    driveTrain.leftEncoder.setPosition(0);
    driveTrain.rightEncoder.setPosition(0);
  }

  public double getCurrentPos() {
    return driveTrain.leftEncoder.getPosition();
  }

  public boolean reachedTarget(double tolerance) {
    return Math.abs(getCurrentPos() - targetPosition) <= tolerance;
  }


}

