package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    // motors
    private CANSparkMax m0CanSparkMax = new CANSparkMax(0, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax m1CanSparkMax = new CANSparkMax(1, MotorType.kBrushless);

    private CANSparkMax m2CanSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax m3CanSparkMax = new CANSparkMax(3, MotorType.kBrushless);

    // drive trains
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(m0CanSparkMax, m1CanSparkMax);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(m2CanSparkMax, m3CanSparkMax);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    // now for the encoders
    private final Encoder leftEncoder = new Encoder(0, 1);
    private final Encoder rightEncoder = new Encoder(2, 3);

    // usually paired together to access the robot's locaition/physics info
    private final DifferentialDriveOdometry odometry;
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDriveWheelSpeeds wheelSpeeds;
    // more info here:
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
    // private ChassisSpeeds chasisSpeeds = new ChassisSpeeds();
    // u don't use them here, but inside the command if needed to convert to
    // differential drive speeds

    AHRS gyro = new AHRS();

    public DriveTrain() {
        leftEncoder.setDistancePerPulse(1);
        rightEncoder.setDistancePerPulse(1);

        odometry = new DifferentialDriveOdometry(gyro.getAngle(), leftEncoder, rightEncoder);
        kinematics = new DifferentialDriveKinematics(0.5);
    }

    public CommandBase drive(double speed, double rotation) {
        return new CommandBase() {
            @Override
            public void initialize() {
                drive.arcadeDrive(speed, rotation);
            }

            @Override
            public void execute() {
                drive.arcadeDrive(speed, rotation);
            }

            @Override
            public void end(boolean interrupted) {
                drive.arcadeDrive(0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase turnLeftCommand(double speed) {
        return new CommandBase() {
            @Override
            public void initialize() {
                drive.arcadeDrive(0, speed);
            }

            @Override
            public void execute() {
                drive.arcadeDrive(0, speed);
            }

            @Override
            public void end(boolean interrupted) {
                drive.arcadeDrive(0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase turnRightCommand(double speed) {
        return new CommandBase() {
            @Override
            public void initialize() {
                drive.arcadeDrive(0, -speed);
            }

            @Override
            public void execute() {
                drive.arcadeDrive(0, -speed);
            }

            @Override
            public void end(boolean interrupted) {
                drive.arcadeDrive(0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase adjustAngleToZero(double adjustingSpeed) {
        return new CommandBase() {
            @Override
            public void initialize() {
                drive.arcadeDrive(0, 0);
            }

            @Override
            public void execute() {
                if (gyro.getAngle() > 0) {
                    drive.arcadeDrive(0, -adjustingSpeed);
                } else if (gyro.getAngle() < 0) {
                    drive.arcadeDrive(0, adjustingSpeed);
                } else {
                    drive.arcadeDrive(0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                drive.arcadeDrive(0, 0);
            }

            @Override
            public boolean isFinished() {
                return gyro.getAngle() == 0;
            }
        };
    }

    public CommandBase driveToDistance(double distance, double speed) {
        return new CommandBase() {
            @Override
            public void initialize() {
                drive.arcadeDrive(0, 0);
            }

            @Override
            public void execute() {
                if (leftEncoder.getDistance() < distance) {
                    drive.arcadeDrive(speed, 0);
                } else {
                    drive.arcadeDrive(0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                drive.arcadeDrive(0, 0);
            }

            @Override
            public boolean isFinished() {
                return leftEncoder.getDistance() >= distance;
            }
        };
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getAngle(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }
}
