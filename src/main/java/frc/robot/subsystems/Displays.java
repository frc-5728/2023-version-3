// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class Displays extends TrapezoidProfileSubsystem {
  // handles the shuffleboard and other user input displays
  // using trapezoid bc maybe motion/drivetrain needs them (?)
  
  /** Creates a new Displays. */
  public Displays() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        0);
  }

  public void tunePID(String name) {
    GenericEntry kP_entry = Shuffleboard.getTab(name).add("kP", 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
  }
}
