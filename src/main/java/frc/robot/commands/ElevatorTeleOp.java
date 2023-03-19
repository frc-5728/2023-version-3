// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorTeleOp extends CommandBase {
  Elevator elevator;
  Joystick joystick = new Joystick(RobotMap.JOYSTICK_BUTTON_PORT);

  /** Creates a new ElevatorTeleOp. */
  public ElevatorTeleOp(Elevator elevator) {
    this.elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getRawButton(3)) {
      elevator.setSpeed(1);
    } 
    else if (joystick.getRawButton(2)) {
      elevator.setSpeed(-1);
    } else {
      elevator.setSpeed(0);
    }
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
