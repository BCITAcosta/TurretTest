// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Turret sub_Turret;
  private static Vision sub_Vision;
  private final SparkMax turretSpark;

  //Nick Maybe work PID
  private final SparkClosedLoopController m_controller, m_turnController;

  private final SparkFlex turretSpinMotor1;
  private final SparkFlexConfig turretSpin1Config;

  private final SparkFlex turretSpinMotor2;
  private final SparkFlexConfig turretSpin2Config;
  
  private final SparkMaxConfig turretSparkConfig;

  private double turretAngleTarget;

  public Turret() {
    turretSpark = new SparkMax(2, MotorType.kBrushless);
    turretSparkConfig = new SparkMaxConfig();
    turretSparkConfig.smartCurrentLimit(40);
    turretSparkConfig.idleMode(IdleMode.kBrake);
    turretSparkConfig.encoder.positionConversionFactor(0.0088235);
    turretSparkConfig.closedLoop.pid(6,0,0.4);
    turretSparkConfig.closedLoop.outputRange(-1, 1);
    turretSparkConfig.closedLoop.positionWrappingEnabled(true);
    turretSparkConfig.closedLoop.positionWrappingInputRange(0, 0.75);
    turretSpark.configure(turretSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //turretSpark.getEncoder().setPosition(0);
    m_turnController = turretSpark.getClosedLoopController();

    turretSpinMotor1 = new SparkFlex(3, MotorType.kBrushless);
        //Nick Maybe work PID
    m_controller = turretSpinMotor1.getClosedLoopController();    
    
    turretSpin1Config = new SparkFlexConfig();
    turretSpin1Config.smartCurrentLimit(80);
    turretSpin1Config.idleMode(IdleMode.kCoast);
    turretSpin1Config.inverted(true);
    // turretSpin1Config.closedLoop.pid(0.00038, 0, 0).outputRange(0.01, 1);
   
    //turretSpin1Config.closedLoop.feedForward.kS(1);
    turretSpin1Config.closedLoop.pid(0.0006,0.000000175,0.0003);
    turretSpin1Config.closedLoop.outputRange(0, 1);
    turretSparkConfig.closedLoop.feedForward.kS(0.01);
    turretSpin1Config.closedLoop.maxMotion
                        .maxAcceleration(24000).allowedProfileError(1);
    turretSpinMotor1.configure(turretSpin1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretSpinMotor2 = new SparkFlex(4, MotorType.kBrushless);

    turretSpin2Config = new SparkFlexConfig();
    turretSpin2Config.smartCurrentLimit(80);
    turretSpin2Config.idleMode(IdleMode.kCoast);
    turretSpin2Config.follow(3, true);
    turretSpinMotor2.configure(turretSpin2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretAngleTarget = 0.0;
    sub_Vision = Vision.getInstance();
  }

  public static Turret getInstance(){
    if (sub_Turret == null){
      sub_Turret = new Turret();
    }
    return sub_Turret;
  }

  private double getRPM(){
    return turretSpinMotor1.getEncoder().getVelocity();
  }

  public void spin(){
    turretSpinMotor1.set(-1);
  }

  public void stopSpin(){
    turretSpinMotor1.set(0);
  }

  public void trimTurret(double trimAmount){
    turretAngleTarget += trimAmount;
  }

  public Command targetTurret(){
    return run(()->{
      turretAngleTarget = turretSpark.getEncoder().getPosition() + (sub_Vision.getTargetYaw()/360);
      if(sub_Vision.targetAcquired())
      {
        m_turnController.setSetpoint(turretAngleTarget, ControlType.kPosition);
      }
    });
  }

  public Command centerTurret(){
     return run(()->{m_turnController.setSetpoint(0.0, ControlType.kPosition);
     });
  }

  // public Command turretScan(){
  //   return run(()->{
  //     if(!sub_Vision.targetAcquired()){
  //       m_turnController.setSetpoint(10, ControlType.kVelocity);
  //     }
  //   });
  // }

  public Command spinUpTest(){
    return runOnce(()->{
        m_controller.setSetpoint(4000, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    });
  }

  public Command stopSpinCommand(){
    return runOnce(()->{
        m_controller.setSetpoint(500, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    });
  }

  public Command runTurretTest(){
    return run(()->{turretSpark.set(1);});
  }

  public Command stopTurretTest(){
    return run(()->{turretSpark.set(0);});
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Nick maybe work PID
    SmartDashboard.putNumber("RPM", getRPM());
    SmartDashboard.putNumber("motor 1",turretSpinMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("motor 2", turretSpinMotor2.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret Position", turretSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret Angle",turretSpark.getEncoder().getPosition()*360);
    SmartDashboard.putNumber("Turret Target Position", turretAngleTarget);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
