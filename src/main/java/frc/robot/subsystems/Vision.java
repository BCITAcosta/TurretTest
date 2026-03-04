package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Vision extends SubsystemBase{
    private static Vision sub_Vision;

    private Spark blinkinDIO;
    private PhotonCamera camera_one;
    private PhotonTrackedTarget target;
    private double targetYaw;
    private boolean b_targetAcquired;

    PIDController targetingController;

    public Vision (){
        targetYaw = 0.0;
        b_targetAcquired = false;
        camera_one = new PhotonCamera("AnomCAM_1");
        camera_one.setFPSLimit(30);
        blinkinDIO = new Spark(1);
    }

    public static Vision getInstance(){
        if (sub_Vision == null){
            sub_Vision = new Vision();
        }
        return sub_Vision;
    }

    private void idleColor(){
        blinkinDIO.set(0.87);
    }

    private void targetAcquiredColor(){
        blinkinDIO.set(0.35);
    }

    public double getTargetYaw(){
        return targetYaw;
    }

    public boolean targetAcquired(){
        return b_targetAcquired;
    }
    private void targeting(){
        var results = camera_one.getAllUnreadResults();
        if(!results.isEmpty()){
            var result = results.get(results.size() - 1);
            if(result.hasTargets()){
                for(var target :result.getTargets()){
                    if(target.getFiducialId() == 12){
                        b_targetAcquired = true;
                        targetYaw = target.getYaw();
                        targetAcquiredColor();
                    // }else if(target.getFiducialId()==6){
                    //     blinkinDIO.set(0.81);
                    }
                }
            }else{
                targetYaw = 0.0;
                idleColor();
            }
        }
    }

    public Command enableTargeting(){
        return run(()->{
            targeting();
        });
    }

    @Override
    public void periodic(){
        SmartDashboard.getNumber("Target Yaw", targetYaw);
    }
}
