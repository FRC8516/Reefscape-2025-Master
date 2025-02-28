// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CalibrationSettings;
import frc.robot.Constants.ClawPositions;
import frc.robot.Constants.ManipulatorConstants;

public class ClawWrist extends SubsystemBase {
      /* Hardware */
      private final TalonFX m_ClawWraistMotor = new TalonFX(ManipulatorConstants.kIntakeWraistMotor, "rio");
      //Motion Magic
      private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
      //backup key values not returned from perference table on shuffleboard....100:1 Gear box
      final double ClawPositionHome = 0.05;
      final double ClawPositionTransfer = 50.0;
      final double ClawPositionLoading = 0.2;
      final double ClawPositionScoring = 30.0;
      final double ClawPositionAlgae = 120.0;
      final double ClawPositionScoringAlgae = 155.0;
      //Use to get from the preference table (Key value)
      final String ClawHomeKey = "Claw Home Position";
      final String ClawScoringKey = "Claw Scoring Position";
      final String ClawTransferKey = "Claw Transfer Postion";
      final String ClawLoadingKey = "ClawLoadingPosition";
      final String ClawAlgaeKey = "Claw Algae Key";
      final String ClawScoringAlgaeKey = "Claw Algae Scoring Key";
      //local setpoint for moving to position by magic motion
      private double setPoint;
      private double backUp;
      private String Key;
      /* Keep a brake request so we can disable the motor */
      private final NeutralOut m_brake = new NeutralOut();
      private double scale = 360;
      //local variable to keep track of position
      StatusSignal<Angle> aCurrentPosition;

  /** Creates a new ClawWraist. */
  public ClawWrist() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    //Used for the homing of the mech (Enable if or when we use it!)
    //configs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    //configs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    //configs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
    m_ClawWraistMotor.setPosition(0);
    //Software limits - forward motion
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 180;  // *Need to check!!!
    /** *********************************************************************************************
     * Motion Magic
    /* Configure current limits   */
    MotionMagicConfigs mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = CalibrationSettings.WraistCalibrations.kCruiseVelocity;  // 5 rotations per second cruise
    mm.MotionMagicAcceleration = CalibrationSettings.WraistCalibrations.kMaxAccelerationMotionMagic; // Target acceleration of 400 rps/s (0.25 seconds to max)
    mm.MotionMagicJerk = CalibrationSettings.WraistCalibrations.kClawJerk; // Target jerk of 4000 rps/s/s (0.1 seconds)

    Slot0Configs slot0 = configs.Slot0;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = CalibrationSettings.WraistCalibrations.kClawkS;   // Add 0.25 V output to overcome static friction
    slot0.kV = CalibrationSettings.WraistCalibrations.kClawkV;   // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = CalibrationSettings.WraistCalibrations.kClawkA;   // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = CalibrationSettings.WraistCalibrations.kClawkP;   // An error of 1 rps results in 0.11 V output
    slot0.kI = CalibrationSettings.WraistCalibrations.kClawkI;   // no output for integrated error
    slot0.kD = CalibrationSettings.WraistCalibrations.kClawkD;   // no output for error derivative
    
    FeedbackConfigs fdb = configs.Feedback;
    fdb.SensorToMechanismRatio = 100;
    //Set Brake mode
    m_ClawWraistMotor.setControl(m_brake);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_ClawWraistMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     //Updates position on the dashboard
     aCurrentPosition = m_ClawWraistMotor.getPosition();
     SmartDashboard.putNumber("Wraist Position", aCurrentPosition.getValueAsDouble());
  }

  //Call by the commands to move claw wraist to positions
  public void MoveClawToPosition(String sMoveTo) {
    //set up the grab from values at Smart Dashboard perference table
    switch (sMoveTo) {
      case ClawPositions.ScoringPosition:;
        //Scoring Position
        backUp = ClawPositionScoring;
        Key = ClawScoringKey;
        break;
      case ClawPositions.TransferPosition:;
        //Claw Transfer Position
        backUp = ClawPositionTransfer;
        Key = ClawTransferKey;
        break;
      case ClawPositions.HomePosition:;
        //Move to home position
        backUp = ClawPositionHome;
        Key = ClawHomeKey;
        break;
      case ClawPositions.AlgaePosition:;
        //move to Algae Pick Position
        backUp = ClawPositionAlgae;
        Key = ClawAlgaeKey;
        break;
      case ClawPositions.AlgaeScoringPosition:;
        //move to Algae Pick Position
        backUp = ClawPositionScoringAlgae;
        Key = ClawScoringAlgaeKey;
        break;
    }
    //gets the current value
	  setPoint = getPreferencesDouble(Key, backUp);
    //sets the new position to the motor controller.
	  this.MoveToPosition(setPoint/scale);
  }

  private void MoveToPosition(double targetPos) {
    /* Use voltage position */
     m_ClawWraistMotor.setControl(m_mmReq.withPosition(targetPos).withSlot(0));
  }

  //This checks Current positon to setpoint for the commands calls - isFinished flag
  public Boolean isClawWraistInPosition() {
   double dError = aCurrentPosition.getValueAsDouble() - setPoint;
   //Returns the check to see if the elevator is in position
   if ((dError < 0.005) || (dError > -0.005)) {
     return true;
   } else {
     return false;
   }
  }

  /**
    * Retrieve numbers from the preferences table. If the specified key is in
    * the preferences table, then the preference value is returned. Otherwise,
    * return the backup value, and also start a new entry in the preferences
    * table.
	 * @return 
    */
    private double getPreferencesDouble(String key, double backup) {
      if (!Preferences.containsKey(key)) {
        Preferences.initDouble(key, backup);
        Preferences.setDouble(key, backup);
      }
      return Preferences.getDouble(key, backup);
      }
}
