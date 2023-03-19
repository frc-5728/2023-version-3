package frc.robot.commands.Automatic;

import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.Move;
import frc.robot.subsystems.Vision.ReflectiveTapeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class autoPeriod extends SequentialCommandGroup {
    
    private final DriveTrain driveTrain;

    private final double disp1 = -8;
    private final double disp2 = 4;

    private final double elevatorHeight = 1;

    public autoPeriod(DriveTrain driveTrain) {

        this.driveTrain = driveTrain;

        addCommands(
                
            new Move(this.disp1, driveTrain),
            new Move(this.disp2, driveTrain),
            new AutoBalance(driveTrain)

        );
        
    }

}
