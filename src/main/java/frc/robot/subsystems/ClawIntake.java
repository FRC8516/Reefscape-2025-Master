// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;

public class ClawIntake extends SubsystemBase {
  /* Hardware */
    private final TalonFX m_ClawIntake = new TalonFX(ManipulatorConstants.kIntakeMotor, "rio");
    private final CANrange m_CoralDetection = new CANrange(OIConstants.CANRangeID, "rio");
    private Timer m_timer = new Timer();
    private boolean isCoralDetected = false;
    private boolean isAfterDetectionRunning = false;
    private boolean isCommandDone = true;
    
  /** Creates a new ClawIntake. */
  public ClawIntake() {
    /* Factory default hardware to prevent unexpected behavior */
    TalonFXConfiguration configs = new TalonFXConfiguration();
      //Set configurations  
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_ClawIntake.getConfigurator().apply(configs);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        
    if (m_CoralDetection.getDistance().getValueAsDouble() <= OIConstants.CANRangeDetectRange){
      isCoralDetected = true;
      isAfterDetectionRunning = true;
      AfterDetection(); 
    } else{
      isCoralDetected = false;
      isAfterDetectionRunning  = false;
    }
    
    SmartDashboard.putBoolean("Coral Detected?", isCoralDetected);

  }

  public void Intake(){
    isCommandDone = false;
    if (isCoralDetected == false && isAfterDetectionRunning == false) {
      m_ClawIntake.setVoltage(ManipulatorConstants.kIntakeVoltage);
    }
  }

  public void Output(){
    m_timer.start();
    while((m_timer.get() < ManipulatorConstants.kIntakeFeedTime) == true){
      m_ClawIntake.setVoltage(ManipulatorConstants.kIntakeVoltage/2);
    }
    StopMotion();
    m_timer.stop();
    m_timer.reset();
  }

  private void AfterDetection() {
    if (isCommandDone == false){
      m_timer.start();
      while((m_timer.get() < 0.25) == true){
        m_ClawIntake.setVoltage(2.0);
      }
      isCommandDone = true;
      StopMotion();
      m_timer.stop();
      m_timer.reset();
    }
  }

  public void StopMotion(){
    m_ClawIntake.stopMotor();
  }

}