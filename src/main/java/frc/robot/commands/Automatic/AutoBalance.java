package frc.robot.commands.Automatic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveTrain;

public class AutoBalance extends CommandBase {

    private final DriveTrain driveTrain;

    boolean autoBalanceXMode = false;
    boolean autoBalanceYMode = false;

    double balanceX;
    double balanceY;

    double balanceThresholdX = 4;
    double balanceThresholdY = 8;

    public AutoBalance(DriveTrain driveTrain) {

        this.driveTrain = driveTrain;

        addRequirements(driveTrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        this.balanceX = driveTrain.gyro.getPitch();
        this.balanceY = driveTrain.gyro.getRoll();

        if (this.balanceX >= Math.abs(this.balanceThresholdX)){
            this.autoBalanceXMode = true;
        }
        else {
            this.autoBalanceXMode = false;
        }

        if (this.balanceY >= Math.abs(this.balanceThresholdY)){
            this.autoBalanceYMode = true;
        }
        else {
            this.autoBalanceYMode = false;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}