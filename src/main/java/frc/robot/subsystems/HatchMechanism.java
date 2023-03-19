// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HatchMechanism extends SubsystemBase {
  // private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
  private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);

  boolean isOpen = false;

  /** Creates a new HatchMechanism. */
  public HatchMechanism() {

  }

  public boolean getIsOpen() {
    return isOpen;
  }

  public void forward() {
    // solenoid.set(Value.kForward);
    if (!isOpen) solenoid.set(isOpen);
    isOpen = true;
  }

  public void reverse() {
    if (isOpen) solenoid.set(isOpen);
    isOpen = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
