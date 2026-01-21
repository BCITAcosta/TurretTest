// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Turret sub_Turret;
  private final SparkMax turretSpark;
  
  private final SparkMaxConfig turretSparkConfig;

  private final DigitalInput dio_Switch;
  
  public Turret() {
    turretSpark = new SparkMax(2, MotorType.kBrushless);
    turretSparkConfig = new SparkMaxConfig();
    turretSparkConfig.smartCurrentLimit(40);
    turretSparkConfig.idleMode(IdleMode.kBrake);
    turretSpark.configure(turretSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    dio_Switch = new DigitalInput(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public static Turret getInstance(){
    if (sub_Turret == null){
      sub_Turret = new Turret();
    }
    return sub_Turret;
  }

  public void turn(){
    turretSpark.set(1);
  }

  public void stop(){
    turretSpark.set(0);
  }

  public Command runTurretTest(){
    return run(()->{
      if(dio_Switch.get()){
        turn();
      }else{
        stop();
      }
    });
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
