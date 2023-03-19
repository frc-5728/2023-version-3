// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorTeleOp;

public class Elevator extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(RobotMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkMaxPIDController pidController = motor.getPIDController();
  private boolean manualMode;
  private double targetPosition;
  /** Creates a new Elevator. */
  public Elevator() {
    SmartDashboard.putBoolean("Manual Mode", true);
    SmartDashboard.putNumber("ArmPosPID", targetPosition);
    pidController.setP(1);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    encoder.setPositionConversionFactor(1);

    
    //TODO: remove after set
    motor.burnFlash();
    setDefaultCommand(new ElevatorTeleOp(this));
  }

  public void setSpeed(double percentOutput) {
    if(!manualMode) return;
    motor.set(percentOutput);
  }

  public void resetEncoder() {
    System.out.println("reset encoder");
    encoder.setPosition(0);
  }

  public double getCurrentPos() {
    return encoder.getPosition();
  }

  public boolean reachedTarget(double tolerance) {
    return Math.abs(getCurrentPos() - targetPosition) <= tolerance;
  }

  public void setPosition(double position) {
    if(manualMode) return;
    targetPosition = position;
    pidController.setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    targetPosition = SmartDashboard.getNumber("ArmPosPID", targetPosition);
    this.manualMode = SmartDashboard.getBoolean("Manual Mode", true);
    SmartDashboard.putNumber("Encoder Value", getCurrentPos());
    setPosition(targetPosition);
  }
}
