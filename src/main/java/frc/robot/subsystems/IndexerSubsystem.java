package frc.robot.subsystems;

import static frc.robot.Constants.indexerconstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{
    private final SparkMax indexermotor;
    public double commandVoltage;
    public IndexerSubsystem() {
        indexermotor = new SparkMax(indexerconstants.kIndexerCanId, MotorType.kBrushless);
    }
    private final Alert  motordisconnect = new Alert("Indexer motor disconnected", Alert.AlertType.kWarning);

    public void periodic() {
        // use for logging and elastic in the future
    }
    public void setVoltage(double volts) {
        indexermotor.setVoltage(volts);
    }
    public double getvoltage(){
        return -indexermotor.getBusVoltage();
    }
    public void index(){
        setVoltage(indexerconstants.feedVolts);
        
    }

    public void reverse(){
        setVoltage(indexerconstants.reverseVolts);
    }

    public void stopindexer(){
        setVoltage(0);
        
    }

}