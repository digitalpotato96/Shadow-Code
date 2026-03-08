package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class MainCommands {

    private ShooterSubsystem Shooter;
    private ClimberSubsystem Climber;
    private FeederSubsystem Feeder;

    public MainCommands(
        ShooterSubsystem Shooter, 
        ClimberSubsystem Climber,
        FeederSubsystem Feeder){
            this.Shooter = Shooter;
            this.Climber = Climber;
            this.Feeder = Feeder;

        }
    


public Command shoot(){ 
    return Shooter.set(-0.50).withTimeout(0.5)                           //Velocity(RPM.of(-1800)).withTimeout(0.5)
        .alongWith(Feeder.set(-1));
    }

public Command suck(){
    return Shooter.set(0.50).withTimeout(0.5)                           //Velocity(RPM.of(1800))
    .alongWith(Feeder.set(1));
    }

public Command Climber_Forward(){
    return Climber.set(-0.25); 
    }

    
public Command Climber_Back(){
    return Climber.set(0.25); 
    }

}