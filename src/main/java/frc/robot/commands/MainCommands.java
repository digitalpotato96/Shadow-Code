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


public Command shoot(){ 
    return Shooter.setVelocity(RPM.of(5000)).withTimeout(1)
        .andThen(Shooter.setVelocity(RPM.of(5000))
        .alongWith(Feeder.set(1)));
    }

public Command suck(){
    return Shooter.setVelocity(RPM.of(5000))
    .alongWith(Feeder.set(-1));
}
}