// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

public class CANClimbSubsystem extends SubsystemBase {
  private final SparkMax climbMotor;

  /** Creates a new CANClimbSubsystem */
  @SuppressWarnings("removal")
  public CANClimbSubsystem() {

    climbMotor = new SparkMax(CLIMB_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the climb motor, set a current limit and apply
    // the config to the controller
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMB_MOTOR_CURRENT_LIMIT);
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", CLIMB_RAISE_VOLTAGE);
    SmartDashboard.putNumber("Intaking feeder roller value", CLIMB_LOWER_VOLTAGE);
  }

  // A method to set the voltage of the climb Motor
  public void raiseClimb(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  public void lowerClimb(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  // A method to stop the rollers
  public void stop() {
    climbMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
