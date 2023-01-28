package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class FollowLimelight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SwerveDrive m_SwerveDrive;
  private Limelight m_limelight;

  private double kP_rotation = -0.05;
  private double kP_forwardDistance = 0.3;
  private double kP_sideDistance = 0.1;
  private double kD_rotation = 0;
  private double kD_forwardDistance = 0;
  private double kD_sideDistance = 0;

  private PIDController controllerForward = new PIDController(kP_forwardDistance, 0, kD_forwardDistance);
  private PIDController controllerSide = new PIDController(kP_sideDistance, 0, kD_sideDistance);
  private PIDController controllerRot = new PIDController(kP_rotation, 0, kD_rotation);
  




  private double rotation;
  private double sideDistance;
  private double forwardDistance;

  private double xVel = 0;
  private double yVel = 0;

  private double rotationCutOff = 0.5;
  private double sideDistanceCutOff = 0.1;
  private double desiredForwardDistance = 3;
  private double forwardDistanceCutOff = 0.5;

  private double[] target;

  ShuffleboardTab tabLL = Shuffleboard.getTab("Limelight");    
  GenericEntry kP_rotEntry = tabLL.add("kP rot LL", kP_rotation).getEntry();
  GenericEntry kP_sideEntry = tabLL.add("kP side LL", kP_sideDistance).getEntry();
  GenericEntry kP_forEntry = tabLL.add("kP forward LL", kP_forwardDistance).getEntry();
  GenericEntry kD_rotEntry = tabLL.add("kD rot LL", kD_rotation).getEntry();
  GenericEntry kD_sideEntry = tabLL.add("kD side LL", kD_sideDistance).getEntry();
  GenericEntry kD_forEntry = tabLL.add("kD forward LL", kD_forwardDistance).getEntry();
  GenericEntry txEntry = tabLL.add("tx", 0).getEntry();
  GenericEntry taEntry = tabLL.add("ta", 0).getEntry();
  GenericEntry headingEntry = tabLL.add("heading", 0).getEntry();

  public FollowLimelight(SwerveDrive sd, Limelight limelight) {
    m_SwerveDrive = sd;
    m_limelight = limelight;

//    ShuffleboardTab tabLL = Shuffleboard.getTab("Limelight");
    Shuffleboard.selectTab("Limelight");
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sd, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP_rotation=kP_rotEntry.getDouble(0);
    kP_forwardDistance=kP_forEntry.getDouble(0);
    kP_sideDistance=kP_sideEntry.getDouble(0);

    kD_rotation=kD_rotEntry.getDouble(0);
    kD_forwardDistance=kD_forEntry.getDouble(0);
    kD_sideDistance=kD_sideEntry.getDouble(0);

    controllerForward.setP(kP_forwardDistance);
    controllerSide.setP(kP_sideDistance);
    controllerRot.setP(kP_rotation);
    controllerForward.setD(kD_forwardDistance);
    controllerSide.setD(kD_sideDistance);
    controllerRot.setD(kD_rotation);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = m_limelight.getVisionTarget();
    double heading = m_SwerveDrive.heading;

 
//    System.out.println("Tx: "+target[1]+" R: "+rotation+" H: "+m_SwerveDrive.heading);
    //FIELD CENTRIC MOTION - MOVING FORWARD TOWARD APRILTAG
    
    if (target[0] == 1){
      double tx=target[0]; 
      double ta=target[3];
      double rotOutput = controllerRot.calculate(heading, 0);
      double sideOutput = controllerSide.calculate(tx, 0);
      double forwardOutput = controllerForward.calculate(ta, (desiredForwardDistance*Math.cos(heading))); 
      m_SwerveDrive.setMotors(forwardOutput, sideOutput, rotOutput);
      
      txEntry.setDouble(tx);
      taEntry.setDouble(ta);
      headingEntry.setDouble(heading);
    }
      
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    m_SwerveDrive.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (target[1] <= rotationCutOff && desiredForwardDistance-target[3] <= forwardDistanceCutOff){
      return true;
    }
    return false;
  }
}
