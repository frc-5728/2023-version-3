package frc.robot.commands;

import frc.robot.subsystems.Vision.ReflectiveTapeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class DropCone extends CommandBase {
    
    private final ReflectiveTapeSubsystem rtSubsystem;

    private final double displacementCone = 0.2;
    private final double elevatorHeight = 1;

    public DropCone(ReflectiveTapeSubsystem rtSubsystem, DriveTrain driveTrain) {

        this.rtSubsystem = rtSubsystem;

        if (rtSubsystem.hasTarget) {

            // Change values to correct units

            // parallel command to raise elevator arm
            addParallel(new TurnLeft(driveTrain, rtSubsystem.getYaw()) );
            addSequential(new Move(rtSubsystem.getRange() - displacementCone, driveTrain) );
            // sequential command to release cone

        }
        else {
            System.out.println("No target");
        }
        
        addRequirements(rtSubsystem);

    }

    @Override
    public void initialize() {
        System.out.println("Cone Drop command initialized");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Cone Drop command ended");
    }

    @Override
    public void isFinished() {
        return false;
    }

}
