// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Elevator extends PIDSubsystem {
  private final CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  
  /** Creates a new Elevator. */
  public Elevator() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.3, 0, 0));
        
        setSetpoint(10);

        encoder.setPositionConversionFactor(1);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    System.out.println("Elevator output: " + output);
    SmartDashboard.putNumber("Elevator output", output);
    System.out.println("Elevator Encoder value: " + encoder.getPosition());
    SmartDashboard.putNumber("Elevator Encoder value", encoder.getPosition());
    System.out.println();

    // motor.set(MathUtil.clamp(output, -0.5, 0.5));
    // motor.set(1);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getPosition();
  }
}
