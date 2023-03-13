// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.OperatorConstants;

public class Arm extends SubsystemBase {
  XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);

  VictorSPX drawer = new VictorSPX(RobotMap.DRAWER_ID);
  VictorSPX arm = new VictorSPX(RobotMap.ARM_MOTOR_ID);
  
  /** Creates a new Arm. */
  public Arm() {
    drawer.set(VictorSPXControlMode.PercentOutput, 0.3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drawer.set(ControlMode.Position, 1);
  }
}
