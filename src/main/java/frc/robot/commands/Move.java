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
public class Move extends PIDCommand {
  private double distance;
  private DriveTrain driveTrain;
  
  /** Creates a new Move. */
  public Move(double distance, DriveTrain driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(0.5, 0.1, 1),
        // This should return the measurement
        () -> driveTrain.gyro.getDisplacementX(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          // Use the output here
          // System.out.println("Move output: " + output);
          System.out.println();

          driveTrain.setSpeed(MathUtil.clamp(-output, -0.1, 0.1));
          
          // System.out.println("getdisplacementX gyro: " + driveTrain.gyro.getDisplacementX());
          // System.out.println("getdisplacementY gyro: " + driveTrain.gyro.getDisplacementY());
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    addRequirements(driveTrain);

    getController().setTolerance(0.1);
    getController().setIntegratorRange(-2, 2);

    this.driveTrain = driveTrain;
    this.distance = distance;
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return driveTrain.gyro.getDisplacementX() == distance;
  }
}
