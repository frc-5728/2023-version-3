// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotMap;

public class ArmProfiledPID extends ProfiledPIDSubsystem {
  // private final VictorSPX motor = new VictorSPX(RobotMap.ARM_MOTOR_ID);
  // private final Encoder encoder = new Encoder(0, 0);
  private final ArmFeedforward armFeedforward = new ArmFeedforward(getMeasurement(), getMeasurement(), getMeasurement());

  private final static double kP = 0; // something to be tuned later
  
  /** Creates a new ArmProfiledPID. */
  public ArmProfiledPID() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            kP,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    // motor.set(ControlMode.MotionMagic, output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
