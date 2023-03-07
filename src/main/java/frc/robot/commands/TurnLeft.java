// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnLeft extends PIDCommand {
  private double angle;
  private DriveTrain driveTrain;
  
  /** Creates a new TurnLeft. */
  public TurnLeft(DriveTrain driveTrain, double angle) {
    // turn "angle" degrees to the left
    super(
        // The controller that the command will use
        new PIDController(0.0003, 0.00015, 0.0003),
        // This should return the measurement
        () -> {
          double result = driveTrain.gyro.getYaw();
          // if (driveTrain.gyro.getYaw() < 0) {
          //   result += 180;
          // }
          // return 180-angle + result;
          return result;
        },
        // This should return the setpoint (can also be a constant)
        () -> angle/2,
        // This uses the output
        output -> {
          // Use the output here
          System.out.println("Turn left output: " + output);

          double outputClamped = MathUtil.clamp(output, -0.1, 0.1);
          driveTrain.setLeftSpeed(outputClamped);
          driveTrain.setRightSpeed(-outputClamped);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveTrain);

    driveTrain.gyro.reset();

    getController().setTolerance(5);
    getController().setIntegratorRange(-5, 5);

    this.driveTrain = driveTrain;
    this.angle = angle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
