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
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.ClawWraist;
import frc.robot.subsystems.Elvator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ClawIntake m_ClawIntake = new ClawIntake();
    public final ClawWraist m_ClawWraist = new ClawWraist();
    public final Elvator m_elevator = new Elvator();
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //elevator buttons
        operator.leftTrigger().onTrue(m_elevator.runOnce(() -> m_elevator.MoveElevatorToPosition("HomePosition")));
        operator.leftBumper().onTrue(m_elevator.runOnce(() -> m_elevator.MoveElevatorToPosition("LoadingPosition")));
        operator.povUp().onTrue(m_elevator.runOnce(() -> m_elevator.MoveElevatorToPosition("L1Position")));
        operator.povRight().onTrue(m_elevator.runOnce(() -> m_elevator.MoveElevatorToPosition("L2Position")));
        operator.povDown().onTrue(m_elevator.runOnce(() -> m_elevator.MoveElevatorToPosition("L3Position")));
        operator.povLeft().onTrue(m_elevator.runOnce(() -> m_elevator.MoveElevatorToPosition("L4Position")));

        //Claw Buttons
        operator.rightTrigger().onTrue(m_ClawIntake.runOnce(() -> m_ClawIntake.Intake()));
        operator.rightBumper().onTrue(m_ClawIntake.runOnce(() -> m_ClawIntake.Output()));
        operator.y().onTrue(m_ClawWraist.runOnce(() -> m_ClawWraist.MoveClawToPosition("HomePosition")));
        operator.x().onTrue(m_ClawWraist.runOnce(() -> m_ClawWraist.MoveClawToPosition("ScoringPosition")));
        operator.a().onTrue(m_ClawWraist.runOnce(() -> m_ClawWraist.MoveClawToPosition("TransferPostion")));
        operator.b().onTrue(m_ClawWraist.runOnce(() -> m_ClawWraist.MoveClawToPosition("LoadingPosition")));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}