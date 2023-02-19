package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

public class DriveTrain extends SubsystemBase {
    // motors
    private CANSparkMax m0CanSparkMax = new CANSparkMax(RobotMap.MOTOR_LEFT0_ID, MotorType.kBrushless);
    private CANSparkMax m1CanSparkMax = new CANSparkMax(RobotMap.MOTOR_LEFT1_ID, MotorType.kBrushless);

    private CANSparkMax m2CanSparkMax = new CANSparkMax(RobotMap.MOTOR_RIGHT0_ID, MotorType.kBrushless);
    private CANSparkMax m3CanSparkMax = new CANSparkMax(RobotMap.MOTOR_RIGHT1_ID, MotorType.kBrushless);

    // drive trains
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(m0CanSparkMax, m1CanSparkMax);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(m2CanSparkMax, m3CanSparkMax);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    // now for the encoders
    private final Encoder leftEncoder = new Encoder(0, 1);
    private final Encoder rightEncoder = new Encoder(2, 3);

    // https://www.youtube.com/watch?v=_mKlRbapkXo&ab_channel=EastRobotics
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    public final PIDController turnController;

    // usually paired together to access the robot's locaition/physics info
    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    AHRS gyro = new AHRS();

    public DriveTrain() {
        leftEncoder.setDistancePerPulse(1);
        rightEncoder.setDistancePerPulse(1);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getDistance(),
                rightEncoder.getDistance());
        kinematics = new DifferentialDriveKinematics(0.5);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, -180.0f);
        
        setDefaultCommand(new TankDrive());
    }

    public void pidWrite(double output) {
        setSpeed(output);
    }

    public void setSpeed(double speed) {
        // give the speed in meters per sec written in chassis
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, speed, 0);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        // drive.tankDrive(wheelSpeeds.leftMetersPerSecond,
        // wheelSpeeds.rightMetersPerSecond);
        setLeftSpeed(wheelSpeeds.leftMetersPerSecond);
        setRightSpeed(wheelSpeeds.rightMetersPerSecond);
    }

    public void setLeftSpeed(double speed) {
        leftMotors.set(speed);
    }

    public void setRightSpeed(double speed) {
        rightMotors.set(speed);
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
                gyro.reset();
            }

            @Override
            public boolean isFinished() {
                return gyro.getAngle() == 0;
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
                    drive.tankDrive(0, -adjustingSpeed * gyro.getAngle());
                } else if (gyro.getAngle() < 0) {
                    drive.tankDrive(0, adjustingSpeed * gyro.getAngle());
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
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }
}
