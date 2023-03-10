package frc.robot.commands.VisionUtil;

import frc.robot.subsystems.Vision.ReflectiveTapeSubsystem;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.Move;

public class DropCone extends SequentialCommandGroup {
    
    private final ReflectiveTapeSubsystem rtSubsystem;
    private final DriveTrain driveTrain;

    private final double displacementCone = 0.2;
    private final double elevatorHeight = 1;

    public DropCone(ReflectiveTapeSubsystem rtSubsystem, DriveTrain driveTrain) {

        this.rtSubsystem = rtSubsystem;
        this.driveTrain = driveTrain;

        if (rtSubsystem.hasTarget) {

            // Change values to correct units

            addCommands(
                
                new TurnMove(rtSubsystem, driveTrain, displacementCone)

            );
            // parallel command to raise elevator arm
           
            // sequential command to release cone

        }
        else {
            System.out.println("No target");
        }
        
        addRequirements(rtSubsystem);

    }

}
