package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FeederConstants.*;

public class FeederSubsystem extends SubsystemBase
{
  
  private final SparkMax Feedermotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
 

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