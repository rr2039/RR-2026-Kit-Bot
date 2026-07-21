// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX feederRoller;
  private final TalonFX intakeLauncherRoller;

  /** Creates a new CANBallSubsystem. */
  @SuppressWarnings("removal")
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID);
    feederRoller = new TalonFX(FEEDER_MOTOR_ID);
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 1; // An error of 1 rps results in 0.7 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    var fslot0Configs = new Slot0Configs();
    fslot0Configs.kS = 0; // Add 0.1 V output to overcome static friction
    fslot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    fslot0Configs.kP = 1; // An error of 1 rps results in 0.7 V output
    fslot0Configs.kI = 0; // no output for integrated error
    fslot0Configs.kD = 0; // no output for error derivative
    
    intakeLauncherRoller.getConfigurator().apply(slot0Configs);
    feederRoller.getConfigurator().apply(fslot0Configs);
    

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    


    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double velocity) {
    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    intakeLauncherRoller.setControl(m_request.withVelocity(velocity).withFeedForward(0));
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double velocity) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    feederRoller.setControl(m_request.withVelocity(velocity).withFeedForward(0));

  }

  // A method to stop the rollers
  public void stop() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    intakeLauncherRoller.setControl(m_request.withVelocity(0).withFeedForward(0));
    intakeLauncherRoller.stopMotor();
    feederRoller.setControl(m_request.withVelocity(0).withFeedForward(0));
    feederRoller.stopMotor();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
