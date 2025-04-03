// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.DriverCalibrations;
import frc.robot.Calibrations.FieldCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;



/**
 * AlignToTag command.
 */
public class TranslationAlignToTag extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private int m_branch;
    private double m_xspeed;
    private double m_yspeed;
    private double m_targetTx;
    private double m_currentTx;
    private double m_errorTx;
    private double m_targetTy;
    private double m_currentTy;
    private double m_errorTy;
    private int m_tagId;
    private int m_lockedTagId;
    private boolean m_validTagId;
    @SuppressWarnings("unused")
    private boolean m_onTarget;
    private RobotCentric m_swerveRequest = new RobotCentric().withRotationalDeadband(DriverCalibrations.kmaxSpeed * 0.1);
    private final ProfiledPIDController m_profiledPid = new ProfiledPIDController(
        DriverCalibrations.kAprilTagTranslationXAlignmentKP,
        0.0, 
        DriverCalibrations.kAprilTagTranslationXAlignmentKD,
        new TrapezoidProfile.Constraints(1.0, 0.05));

    /**
     * AlignToTag Constructor.
     *
     * @param drivetrain The drivetrain
     */
    public TranslationAlignToTag(int branch, CommandSwerveDrivetrain drivetrain) {
        m_branch = branch;  // This also matches the pipeline number
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        System.out.println("tag");
    }

    @Override
    public void initialize() {
        System.out.println("intizulaizeing");
        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("pipeline").setDouble(m_branch);
        m_onTarget = false;
        m_targetTx = 0.0;  // Once a valid target is found, use the hashmap to set this target
        m_lockedTagId = 0;  // Init with an invalid AprilTag ID
    }

    @Override
    public void execute() {
        System.out.println("executing");
        m_tagId = (int) NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tid")
                                            .getInteger(69);  // Get the current AprilTag ID
        System.out.println(m_tagId);
        m_validTagId = FieldCalibrations.m_validTagIds.contains(m_tagId);  // Make sure it's a coral reef AprilTag ID

        // Default to doing nothing
        m_xspeed = 0.0;
        m_yspeed = 0.0;

        // Have a valid AprilTag ID
        if (m_validTagId) {
            System.out.println("valid april tag");
            // Lock onto the first valid AprilTag ID
            if (m_lockedTagId == 0) {
                System.out.println("locking...");
                m_lockedTagId = m_tagId;
                m_targetTx = FieldCalibrations.m_coralReefTargets.get(m_lockedTagId).get(m_branch);
            }
            
            // Only servo to the locked AprilTag ID
            if (m_lockedTagId == m_tagId) {
                System.out.println("working...");
                m_currentTx = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tx")
                                                  .getDouble(DriverCalibrations.kLimelightDefaultKTx);
                m_currentTy = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ty")
                                                  .getDouble(DriverCalibrations.kLimelightDefaultKTx);
                m_errorTx = m_currentTx - m_targetTx;
                m_errorTy = m_currentTy - m_targetTy;
                m_xspeed = m_profiledPid.calculate(-m_errorTx);
                m_yspeed = m_profiledPid.calculate(-m_errorTy);//DriverCalibrations.kAprilTagTranslationYRate;
                if (Math.abs(m_errorTx) < DriverCalibrations.kAprilTagTranslationXOnTarget) {
                    System.out.println("on target");
                    m_onTarget = true;
                }
            }
        }

        // Apply the robot-centric translation speeds
        m_drivetrain.setControl(m_swerveRequest.withVelocityX(m_xspeed).withVelocityY(m_yspeed));

    }
    
    @Override
    public void end(boolean interrupted) {
        // m_drivetrain.setControl(m_swerveRequest.withVelocityX(0.0).withVelocityY(0.0));
        
    }

    @Override
    public boolean isFinished() {
        // return m_onTarget;
        return false;
    }
}
