package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CounterweightPIDSubsystem;

public class WeightCalibrationCommand extends CommandBase{
    private CounterweightPIDSubsystem counterweightPIDSubsystem;


    public WeightCalibrationCommand(CounterweightPIDSubsystem newcounterweightPIDSubsystem){
        counterweightPIDSubsystem = newcounterweightPIDSubsystem;

    }

    @Override
    public void initialize(){
        System.out.println(counterweightPIDSubsystem.getEncoder());
        counterweightPIDSubsystem.setMotor(0.2);
    }
    
    @Override
    public void end(boolean interrupted){
        if (!interrupted){
            counterweightPIDSubsystem.setMotor(0);
            counterweightPIDSubsystem.setEncoder(3000);
            System.out.println(counterweightPIDSubsystem.getEncoder());

        }
    }
    @Override
    public boolean isFinished(){

        //System.out.println(counterweightPIDSubsystem.getSwitch());
        return counterweightPIDSubsystem.getSwitch();
        
    }


}
