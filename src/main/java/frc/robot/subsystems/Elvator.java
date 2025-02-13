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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.Constants.ManipulatorConstants;

public class Elvator extends SubsystemBase {
  /* Hardware */
  	private final TalonFX m_ElevatorMotorL = new TalonFX(ManipulatorConstants.kElevatorMotorLeft, "rio");
    private final TalonFX m_ElevatorMotorR = new TalonFX(ManipulatorConstants.kElevatorMotorRight, "rio");
    //Motion Magic
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    //backup key values not returned from perference table on shuffleboard....100:1 Gear box
      final double PositionHome = 0.1;
      final double PositionLoading = 10;
      final double PositionL1 = 5;
      final double PositionL2 = 10;
      final double PositionL3 = 15;
      final double PositionL4 = 20;
      //Use to get from the preference table (Key value)
      final String HomeKey = "Elevator Home Pos";
      final String LoadingKey = "Elevator Loading";
      final String L1Key = "L1 Position";
      final String L2Key = "L2 Position";
      final String L3Key = "L3 Position";
      final String L4Key = "L4 Position";
      //local setpoint for moving to position by magic motion
      private double setPoint;
      private double backUp;
      private String Key;
      /* Keep a brake request so we can disable the motor */
      private final NeutralOut m_brake = new NeutralOut();
      private double scale = 360;
      //local variable to keep track of position
      StatusSignal<Angle> aCurrentPosition;

  /** Creates a new Elvator. */
  public Elvator() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
        //Used for the homing of the mech (Enable if or when we use it!)
    //configs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    //configs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    //configs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
    
    //Software limits - forward motion
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 180;  // *Need to check!!!
    /** *********************************************************************************************
     * Motion Magic
    /* Configure current limits   */
    MotionMagicConfigs mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 6; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 1.5; // Target acceleration of 400 rps/s (0.25 seconds to max)
    mm.MotionMagicJerk = 50; // Target jerk of 4000 rps/s/s (0.1 seconds)

    Slot0Configs slot0 = configs.Slot0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60;   // An error of 1 rps results in 0.11 V output
    slot0.kI = 0;    // no output for integrated error
    slot0.kD = 1;    // no output for error derivative
    
    FeedbackConfigs fdb = configs.Feedback;
    fdb.SensorToMechanismRatio = 16;  //16:1 Gearbox
    // Set the position to 0 rotations for initial use
    m_ElevatorMotorL.setPosition(0);
    m_ElevatorMotorR.setPosition(0);
    //Set Brake mode
    m_ElevatorMotorL.setControl(m_brake);
    m_ElevatorMotorR.setControl(m_brake);
    /* Setup Master/Slave Relation */
    //m_ElevatorMotorR.setControl(m_ElevatorMotorL.)

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_ElevatorMotorL.getConfigurator().apply(configs);
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
     aCurrentPosition = m_ElevatorMotorL.getPosition();
     SmartDashboard.putNumber("Wraist Position", aCurrentPosition.getValueAsDouble());
  }

  //Call by the commands to move claw wraist to positions
  public void MoveElevatorToPosition(String sMoveTo) {
    //set up the grab from values at Smart Dashboard perference table
    switch (sMoveTo) {
      case ElevatorPositions.HomePosition:;
        // Home Position
        backUp = PositionHome;
        Key = HomeKey;
        break;
      case ElevatorPositions.LoadingPosition:;
        // Loading Position
        backUp = PositionLoading;
        Key = LoadingKey;
        break;
      case ElevatorPositions.L1Position:;
        // L1 position
        backUp = PositionL1;
        Key = L1Key;
        break;
      case ElevatorPositions.L2Position:;
        // L2 position
        backUp = PositionL2;
        Key = L2Key;
        break;
      case ElevatorPositions.L3Position:;
        // L3 position
        backUp = PositionL3;
        Key = L3Key;
        break;
      case ElevatorPositions.L4Position:;
        // L4 position
        backUp = PositionL4;
        Key = L4Key;
        break;
    }

    //gets the current value
	  setPoint = getPreferencesDouble(Key, backUp);
    //sets the new position to the motor controller.
	  this.MoveToPosition(setPoint/scale);
  }

  private void MoveToPosition(double targetPos) {
    /* Use voltage position */
     m_ElevatorMotorL.setControl(m_mmReq.withPosition(targetPos).withSlot(0));
  }

  //This checks Current positon to setpoint for the commands calls - isFinished flag
  public Boolean isClawWraistInPosition() {
   double dError = aCurrentPosition.getValueAsDouble() - setPoint;
   //Returns the check to see if the elevator is in position
   if ((dError < 0.5) || (dError > -0.5)) {
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
