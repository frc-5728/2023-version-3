// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// created from this tutorial: https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html
// or official doc

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  private final DriveTrain driveTrain;
  
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain driveTrain, double angle) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          // Use the output here
          System.out.println("Turn to angle output: ");
          System.out.println(output);
        });

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(RobotMap.TURNING_RATE);
        
        this.driveTrain = driveTrain;
        
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
