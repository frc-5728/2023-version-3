// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.commands.HatchMechanismCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetReadyToHatch extends ParallelCommandGroup {
  /** Creates a new GetReadyToHatch. */
  public GetReadyToHatch(Elevator elevator, Arm arm, HatchMechanism hatch) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ElevatorCommand(elevator, 0), new DrawerInOut(arm, 0), new ArmCommand(arm, 0),
        new HatchMechanismCommand(hatch));
  }
}
