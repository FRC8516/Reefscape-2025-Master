// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CalibrationSettings;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;

public class Climber extends SubsystemBase {
  /* Hardware */
    public final TalonFX m_ClimberMotor = new TalonFX(ManipulatorConstants.kClimberMotor, "rio");
      /** Creates a new Climber. */
    private final CommandXboxController operator = new CommandXboxController(
            Constants.OIConstants.kOperatorControllerPort);
  public Climber() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    //Software limits - forward motion
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;  // *Need to check!!!
    //Set configurations  
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0 = configs.Slot0;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = CalibrationSettings.ClimberCalibrations.kClimberkS;   // Add 0.25 V output to overcome static friction
    slot0.kV = CalibrationSettings.ClimberCalibrations.kClimberkV;   // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = CalibrationSettings.ClimberCalibrations.kClimberkA;   // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = CalibrationSettings.ClimberCalibrations.kClimberkP;   // An error of 1 rps results in 0.11 V output
    slot0.kI = CalibrationSettings.ClimberCalibrations.kClimberkI;   // no output for integrated error
    slot0.kD = CalibrationSettings.ClimberCalibrations.kClimberkD;   // no output for error derivative
    
    FeedbackConfigs fdb = configs.Feedback;
    fdb.SensorToMechanismRatio = 100;
    m_ClimberMotor.getConfigurator().apply(configs);
    
  }

  @Override
  public void periodic() {
    if (operator.back().getAsBoolean() == true){
      m_ClimberMotor.setVoltage(MathUtil.applyDeadband(operator.getRightY(), OIConstants.kDriveDeadband) * 7 );
    }else{
      m_ClimberMotor.setVoltage(MathUtil.applyDeadband(operator.getRightY(), OIConstants.kDriveDeadband));
    }
      // This method will be called once per scheduler run
  }
  public void ExtendClimber (Double Controller) {
    
  }
  
}
