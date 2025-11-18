package frc.robot.Commands;

import edu.wpi.first.units.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.limelightvision.limelight.frc.LimeLight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.NoSuchTargetException;
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
  public final String m_direction;
  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.01);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.01);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  
  public double rotationalRate = 0;
  public double velocityX = 0;
  // private static final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, String direction) {
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    this.m_direction = direction;
    addRequirements(m_Limelight);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (m_Limelight.getClosestFiducial() != null){
      if (m_direction =="left"){
        /*align left
          Get closest apriltag and check it based off a list of acceptable tags to align to
        */
      }else if(m_direction == "right"){
        /*align right
          Get closest apriltag and check it based off a list of acceptable tags to align to
        */
      }
    }else{
      throw new NoSuchTargetException("No fiducials found.");
    }
    /*
     if april tag is valid
          drive relative to tag, maintain parallel to tag
          
          if want to move to reef

          Left:
              Move to the left and forward maintain parallel to april tag
          Right:
              move right and forward maintain parallel to april tag

     */
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