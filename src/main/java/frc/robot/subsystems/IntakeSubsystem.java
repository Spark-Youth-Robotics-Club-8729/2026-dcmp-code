package frc.robot.subsystems;

import static frc.robot.Constants.intakeconstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeconstants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

public class IntakeSubsystem {
    private final TalonFX  intakemotor;
    private final SparkMax slapdownmotor;
    private final RelativeEncoder slapdownencoder;
    private final PIDController slapdownPID;
    public double commandVoltage;
    public IntakeSubsystem() {
        intakemotor = new TalonFX(intakeconstants.kRollerID);
        slapdownmotor = new SparkMax(intakeconstants.kSlapdownID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder.positionConversionFactor(
            (2*Math.PI)/intakeconstants.slapdowngearratio);

        slapdownmotor.configure(config,ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        slapdownencoder = slapdownmotor.getEncoder();
        slapdownencoder.setPosition(0);
        slapdownPID = new PIDController(intakeconstants.slapdownUpKp, 0, intakeconstants.slapdownUpKd);

    }

    
    private final Alert  motordisconnect = new Alert("Intake motor disconnected", Alert.AlertType.kWarning);

    public void periodic() {
        // use for logging and elastic in the future
    }
    public void setVoltage(double volts) {
        intakemotor.setVoltage(volts);
    }
    public double getvoltage(){
        return -intakemotor.get();
    }
    public void intake(){
        setVoltage(intakeconstants.rollerIntakeVolts);
        
    }

    public void outtake(){
        setVoltage(intakeconstants.rollerOuttakeVolts);
    }

    public void stopintake(){
        setVoltage(0);
    }

    public void slapdowndown(){

    }

}
