package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
 import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {
    private  double  wheelDiamInches = 4.084;
    private double wheelCircumferenceMaters=wheelDiamInches*Math.PI*0.0254; // 0.32588 
    private double gearRatioDrive=8.1428; 
    private double MPSToRPM = 60.0*gearRatioDrive/wheelCircumferenceMaters;  
    double twoPi=2*Math.PI;
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    Counter absEncoder;
    RelativeEncoder driveEnc, turnEnc;
    SparkMaxPIDController driveController, turnController;
    double absEncConversion=87890.625;  // converts period to degrees
    double absEncConversionRad=87890.625*Math.PI/180.;  // converts period to radians
    String name;   
    double angleOffsetDeg, angleOffsetRev;     
    double driveSet=0,turnSet=0;
    int reverse=1; double speedPrev=0;
    double zeroAngle;
    double[] zeroAngleArray={100.0, -3, 80.0, 73};
    int EncID;

    static double  
            kPturn=3.6/64,
            kDturn=0,
            kPdrive=0.00005,
            kFdrive=0.000175;

    SwerveModule(String _name, int driveID, int turnID){
        name=_name;
        if(_name.equals("FR")){
            EncID=0;
            zeroAngle=zeroAngleArray[0];
        }

        if(_name.equals("FL")){
            EncID=2;
            zeroAngle=zeroAngleArray[1];
        }

        if(_name.equals("BR")){
            EncID=1;
            zeroAngle=zeroAngleArray[2];
        }

        if(_name.equals("BL")){
            EncID=3;
            zeroAngle=zeroAngleArray[3];
        }




        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kCoast);
        if(_name.equals("FR")) driveMotor.setInverted(false);
        if(_name.equals("FL")) driveMotor.setInverted(false);
        if(_name.equals("BL")) driveMotor.setInverted(true);
        if(_name.equals("BR")) driveMotor.setInverted(true);

        driveEnc=driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        driveController.setFF(kFdrive);

        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setInverted(false);
        turnEnc=turnMotor.getEncoder();
        turnEnc.setPosition(0);
        turnController = turnMotor.getPIDController();
        turnController.setP(kPturn);
        absEncoder = new Counter();
        absEncoder.setUpSource(new DigitalInput(EncID));
        absEncoder.setSemiPeriodMode(true);
        Timer.delay(.5);
        double initialAngle=getAbsAngle();
        angleOffsetDeg=zeroAngle-initialAngle;
        angleOffsetRev=angleOffsetDeg/360;
        SmartDashboard.putNumber(name+" zeroPos", zeroAngle);
        SmartDashboard.putNumber(name+" offset", angleOffsetDeg);

        SmartDashboard.putNumber("kP Turn", kPturn);
        SmartDashboard.putNumber("kD Turn", kDturn);
        SmartDashboard.putNumber("kF Drive", kFdrive);
        SmartDashboard.putNumber("kP Drive", kPdrive);

    }

// gets the turn module absolute angle in degrees    
    public double getAbsAngle(){
        return 360-absEncConversion*absEncoder.getPeriod();
    }

public double getAbsAngleRad(){
        return Math.PI-absEncConversionRad*absEncoder.getPeriod();
    }

// gets the turn module relative angle in radians   
public double getTurnPosition_Rad(){
    return turnEnc.getPosition()*2*Math.PI;
}

// gets the turn module relative angle in radians   
public double getRelRot(){
    return turnEnc.getPosition();
}

// gets the drive module distance in rotations    
public double getDriveVelocity(){
    return driveEnc.getVelocity();
}

// gets the drive module distance in rotations    
public double getDrivePosition(){
    return driveEnc.getPosition();
}
  


// set the drive and turn motors    
public void setMotors(double speed, double turnAngle){

    double acc = (speed-speedPrev)/0.020;
    speedPrev=speed;
    driveController.setReference(speed*MPSToRPM, ControlType.kVelocity);  
    SmartDashboard.putNumber("turn Angle in sdm", turnAngle/twoPi);  
    turnController.setReference(turnAngle/twoPi, ControlType.kPosition);    

    SmartDashboard.putNumber(name+" rev", reverse);

//  turnController.setReference(64*(turnSet/(360)+angleOffsetRev), ControlType.kPosition);
}



public static void updateConstants(){
    kFdrive=SmartDashboard.getNumber("kF Drive", 0);
    kPdrive=SmartDashboard.getNumber("kP Drive", 0);
    kPturn=SmartDashboard.getNumber("kP Turn", 0);
    kDturn=SmartDashboard.getNumber("kD Turn", 0);
}


public double MPStoRPM(double mps){
    return mps*MPSToRPM;
}

public void setMotorsAllStop() {
    setMotors(0,0);;

}
public void resetEncoders() {
    driveEnc.setPosition(0);
  }

}