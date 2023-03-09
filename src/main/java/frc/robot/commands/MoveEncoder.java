// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveEncoder extends PIDCommand {
  /** Creates a new MoveEncoder. */
  public MoveEncoder(DriveTrain driveTrain, double distance) {
    super(
        // The controller that the command will use
        new PIDController(0.5, 0.1, 0.5),
        // This should return the measurement
        () -> driveTrain.lefEncoder.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          // Use the output here
          driveTrain.setSpeed(MathUtil.clamp(output, -0.5, 0.5));

          SmartDashboard.putNumber(getName() + " PID Output", output)
        });
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
