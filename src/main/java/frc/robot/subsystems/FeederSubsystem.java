package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class FeederSubsystem extends SubsystemBase
{
  
  private final SparkMax Feedermotor = new SparkMax(4, MotorType.kBrushless);
 

  public FeederSubsystem()
  {
    setDefaultCommand(set(0));
  }
  public Command set(double dutycycle)
  {
    return run(()->Feedermotor.set(dutycycle));
  }

  @Override
  public void simulationPeriodic() {
    //   shooter.simIterate();
  }
}