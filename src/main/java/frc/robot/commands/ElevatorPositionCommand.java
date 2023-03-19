// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorPositionCommand extends CommandBase {
  final Elevator elevator;
  final Position pos;

  public enum Position {
    LOW(-200), MID(-700), HIGH(-1337);

    private final double position;
    Position(double position) {
      this.position = position;
    }

    public double getPosition() {
      return this.position;
    }
  }

  public ElevatorPositionCommand(Elevator elevator, Position pos) {
    this.elevator = elevator;
    this.pos = pos;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPosition(pos.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
