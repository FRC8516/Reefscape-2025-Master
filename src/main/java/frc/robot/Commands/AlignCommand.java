package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.VisionSubsystem;
class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
      super(kP, kI, kD);
  }
  
  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
      super(kP, kI, kD);
      this.setTolerance(tolerance);
  }
}
public class AlignCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_Limelight;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.01);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.01);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  
  public double rotationalRate = 0;
  public double velocityX = 0;
  // private static final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    addRequirements(m_Limelight);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
    RawFiducial fiducial; 

    

    try {
      fiducial = m_Limelight.getFiducialWithId(1/*m_Limelight.tagID*/);

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint()) {
        this.end(true);
      }

      SmartDashboard.putNumber("txnc", fiducial.txnc);
      SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);
      m_drivetrain.setControl(
          alignRequest.withRotationalRate(-rotationalRate).withVelocityX(-velocityX));//.withVelocityY(velocityY));
      // drivetrain.applyRequest(() -> alignRequest.withRotationalRate(0.5 *
      // MaxAngularRate)
      // .withVelocityX(xPidController.calculate(0.2 * MaxSpeed)));
      // drivetrain.setControl(brake);
    } catch (VisionSubsystem.NoSuchTargetException nste) { 
      System.out.println("No apriltag found");
      if ((rotationalRate != 0) && (velocityX != 0)){
        m_drivetrain.setControl(
          alignRequest.withRotationalRate(-rotationalRate).withVelocityX(-velocityX));//.withVelocityY(velocityY));
        }
      }
      
    }
  

  @Override
  public boolean isFinished() {
    return rotationalPidController.atSetpoint() && xPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }
}