package frc.robot.commands.VisionUtil;

import frc.robot.subsystems.Vision.ReflectiveTapeSubsystem;
import frc.robot.subsystems.Vision.AprilTagSubsystem;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.Move;

public class TurnMove extends SequentialCommandGroup {

    public TurnMove(ReflectiveTapeSubsystem rtSubsystem, DriveTrain driveTrain, double displacement) {

        if (rtSubsystem.hasTarget) {

            addCommands(

                new TurnLeft(driveTrain, rtSubsystem.getYaw()),
                new Move(rtSubsystem.getRange() - displacement, driveTrain)

            );

            // Change values to correct units

        }
        else {
            System.out.println("No target");
        }
        
        addRequirements(rtSubsystem);

    }

    public TurnMove(AprilTagSubsystem atSubsystem, DriveTrain driveTrain, double displacement) {

        if (atSubsystem.hasTarget) {

            addCommands(

                new TurnLeft(driveTrain, atSubsystem.getYaw()),
                new Move(atSubsystem.getRange() - displacement, driveTrain)

            );

            // Change values to correct units

        }
        else {
            System.out.println("No target");
        }
        
        addRequirements(atSubsystem);

    }

}