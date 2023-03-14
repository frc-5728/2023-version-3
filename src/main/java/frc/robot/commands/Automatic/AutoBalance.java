// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automatic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends PIDCommand {
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveTrain driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(0.0003, 0.1, 0.1),
        // This should return the measurement
        () -> driveTrain.gyro.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          driveTrain.setSpeed(MathUtil.clamp(output, -0.1, 0.1));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    getController().setTolerance(2);

    addRequirements(driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

/* package frc.robot.commands.Automatic;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends PIDCommand {

    private final DriveTrain driveTrain;

    boolean autoBalanceXMode = false;
    boolean autoBalanceYMode = false;

    double balanceX;
    double balanceY;

    double balanceThresholdX = 4;
    double balanceThresholdY = 8;

    public AutoBalance(DriveTrain driveTrain) {

        this.driveTrain = driveTrain;

        addRequirements(driveTrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        this.balanceX = driveTrain.gyro.getPitch();
        this.balanceY = driveTrain.gyro.getRoll();

        if (this.balanceX >= Math.abs(this.balanceThresholdX)){
            this.autoBalanceXMode = true;
        }
        else {
            this.autoBalanceXMode = false;
        }

        if (this.balanceY >= Math.abs(this.balanceThresholdY)){
            this.autoBalanceYMode = true;
        }
        else {
            this.autoBalanceYMode = false;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

} */