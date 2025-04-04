// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.CoralScoringPositions;
import frc.robot.Commands.TranslationAlignToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.ClawWrist;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private final SendableChooser<Command> m_autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond); // 1/2 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(
            Constants.OIConstants.kDriverControllerPort);
    private final CommandXboxController operator = new CommandXboxController(
            Constants.OIConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ClawIntake m_ClawIntake = new ClawIntake();
    public final ClawWrist m_ClawWrist = new ClawWrist();
    public final Elevator m_elevator = new Elevator();
    public final Climber m_Climber = new Climber();
    public final VisionSubsystem m_Vision = new VisionSubsystem();

    private final CoralScoringPositions m_ScoringPositionL1 = new CoralScoringPositions(m_ClawWrist, m_elevator, "L1");
    private final CoralScoringPositions m_ScoringPositionL2 = new CoralScoringPositions(m_ClawWrist, m_elevator, "L2");
    private final CoralScoringPositions m_ScoringPositionL3 = new CoralScoringPositions(m_ClawWrist, m_elevator, "L3");
   private final CoralScoringPositions m_transfer = new CoralScoringPositions(m_ClawWrist, m_elevator, "clawTransfer");
    public final CoralScoringPositions m_HomePosistion = new CoralScoringPositions(m_ClawWrist, m_elevator, "Home");
    private final CoralScoringPositions m_testingHome = new CoralScoringPositions(m_ClawWrist, m_elevator, "clawHome");
    //private final CoralScoringPositions m_testingTransfer = new CoralScoringPositions(m_ClawWrist, m_elevator,"clawTransfer");
    private final CoralScoringPositions m_testingScoringAlgae = new CoralScoringPositions(m_ClawWrist, m_elevator,
            "clawScoring");
    private final CoralScoringPositions m_testingAlgae = new CoralScoringPositions(m_ClawWrist, m_elevator,
            "clawAlgae");
    private final CoralScoringPositions m_LoadingPosition = new CoralScoringPositions(m_ClawWrist, m_elevator, "Loading");
    private final Command m_intake = m_ClawIntake.runOnce(() -> m_ClawIntake.Intake());
    private final Command m_Outake = m_ClawIntake.runOnce(() -> m_ClawIntake.Output());
    private final Command m_AlgaeIntake = m_ClawIntake.runOnce(() -> m_ClawIntake.intakeAlgae());
    private final Command m_AlgaeOutake = m_ClawIntake.runOnce(() -> m_ClawIntake.OutputAlgae());
    private final Command m_inch = m_ClawIntake.runOnce(() -> m_ClawIntake.inch());
    private final Command m_autoAlgaeintake = m_ClawIntake.runOnce(() -> m_ClawIntake.AutoIntakeAlgae());
    private final Command m_autoOutake = m_ClawIntake.runOnce(() -> m_ClawIntake.AutoOutput());
    private final Command m_autoAlgaeoutake = m_ClawIntake.runOnce(() -> m_ClawIntake.AutoOutputAlgae());
    private final Command m_Stop = m_ClawIntake.runOnce(() -> m_ClawIntake.StopMotion());
    //private final Command m_AlignLeft = new TranslationAlignToTag(1,drivetrain,"left");
    //private final Command m_AlignRight = new TranslationAlignToTag(1, drivetrain, "right");
    public RobotContainer() {
        NamedCommands.registerCommand("Home", m_HomePosistion);
        NamedCommands.registerCommand("Scoring L1", m_ScoringPositionL1);
        NamedCommands.registerCommand("Scoring L2", m_ScoringPositionL2);
        NamedCommands.registerCommand("Scoring L3", m_ScoringPositionL3);
        NamedCommands.registerCommand("Algae", m_testingAlgae);
        NamedCommands.registerCommand("Intake", m_intake);
        NamedCommands.registerCommand("Output", m_autoOutake);
        NamedCommands.registerCommand("AlgaeIntake", m_autoAlgaeintake);
        NamedCommands.registerCommand("AlgaeOutput", m_autoAlgaeoutake);
        NamedCommands.registerCommand("StopMotors", m_Stop);
        NamedCommands.registerCommand("Loading", m_LoadingPosition);
        m_autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive
                // Drive forward with negative Y (forward)
                .withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), OIConstants.kDriveDeadband) * MaxSpeed)
                // Drive left with negitive X (left)
                .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), OIConstants.kDriveDeadband) * MaxSpeed) 
                // Drive counterclockwise with negative X (left)
                .withRotationalRate(-MathUtil.applyDeadband(joystick.getRightX(), OIConstants.kDriveDeadband) * MaxAngularRate)
            ));
           // joystick.b().whileTrue(m_AlignRight);
            //joystick.x().whileTrue(m_AlignLeft);

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.povDown()
                .whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
                        new Rotation2d(-MathUtil.applyDeadband(joystick.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(joystick.getLeftX(), OIConstants.kDriveDeadband)))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        
        // Operator Buttons below\
        operator.y().onTrue(m_testingHome);
        operator.b().onTrue(m_HomePosistion);
        //operator.b().onTrue(m_testingTransfer);
        operator.x().onTrue(m_testingScoringAlgae);
        operator.a().onTrue(m_testingAlgae);

        
        operator.povDown().onTrue(m_LoadingPosition);
        operator.povLeft().onTrue(m_ScoringPositionL1);
        operator.povUp().onTrue(m_ScoringPositionL2);
        operator.povRight().onTrue(m_ScoringPositionL3);
        
        joystick.leftTrigger().onTrue(m_intake);
      //operator.leftTrigger().onTrue(m_intake);

        joystick.rightTrigger().onTrue(m_Outake);
      //operator.rightTrigger().onTrue(m_Outake);
        
        joystick.leftBumper().onTrue(m_AlgaeIntake);
      //operator.leftBumper().onTrue(m_AlgaeIntake);
        
        joystick.rightBumper().onTrue(m_AlgaeOutake);
      //operator.rightBumper().onTrue(m_AlgaeOutake);

      operator.back().onTrue(m_transfer);
        operator.start().onTrue(m_inch);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
