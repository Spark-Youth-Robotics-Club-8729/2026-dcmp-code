package frc.robot.subsystems;

import static frc.robot.Constants.intakeconstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

public class IntakeSubsystem extends SubsystemBase{
    public static int count = 0;
    private final TalonFX  intakemotor;
    private final SparkMax slapdownmotor;
    private final AbsoluteEncoder slapdownencoder;
    private final Debouncer slapdownSettleDebouncer = new Debouncer(0.15, DebounceType.kRising);
    // Detect bumper contact (stall) and stop driving the slapdown PID when we hit a hard stop
    private final Debouncer slapdownStallDebouncer =
            new Debouncer(0.15, DebounceType.kRising);
    private PIDController slapdownPID;
    private double slapdowntarget;
    public double commandVoltage;
    public IntakeSubsystem() {
        intakemotor = new TalonFX(intakeconstants.kRollerID);
        slapdownmotor = new SparkMax(intakeconstants.kSlapdownID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.absoluteEncoder
            .positionConversionFactor((2.0 * Math.PI))
            .velocityConversionFactor((2.0 * Math.PI) / 60)
            .inverted(true);

        slapdownmotor.configure(config,ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        slapdownencoder = slapdownmotor.getAbsoluteEncoder();
        slapdownPID = new PIDController(intakeconstants.slapdownUpKp, 0, 0); //readd derivative and tune later

        slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
        slapdowntarget = slapdownencoder.getPosition();
        slapdownPID.enableContinuousInput(0, 2 * Math.PI);

    }
    
    private final Alert  motordisconnect = new Alert("Intake motor disconnected", Alert.AlertType.kWarning);

    

    public void periodic() {
        double output = slapdownPID.calculate(slapdownencoder.getPosition(), slapdowntarget)*intakeconstants.slapdowngearratio;
        
        // Stall detection: if motor is stalled (high voltage, high current, low velocity), set output to 0
        boolean slapdownStalled = slapdownStallDebouncer.calculate(
            Math.abs(output) >= intakeconstants.slapdownStallAppliedVolts
                && Math.abs(slapdownencoder.getVelocity()) <= intakeconstants.slapdownStallVelocityRadPerSec);
        
        if (slapdownStalled) {
            output = 0;
        }
        
        slapdownmotor.setVoltage(output);

        //System.out.println("current position: " + slapdownencoder.getPosition());
        //System.out.println("target: " + slapdowntarget);

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


    //slapdown 
    public void slapdowndown(){
        //sets the goal and the new PID
        slapdowntarget = intakeconstants.slapdownDownAngleRad;
        slapdownPID = new PIDController(intakeconstants.slapdownDownKp, intakeconstants.slapdownDownKi, intakeconstants.slapdownDownKd);
        slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
        slapdownPID.enableContinuousInput(0, 2 * Math.PI);
        count = 1;
    }
    public void slapdownup(){
        //sets the goal and the new PID
        slapdowntarget = intakeconstants.slapdownUpAngleRad;
        slapdownPID = new PIDController(intakeconstants.slapdownUpKp, intakeconstants.slapdownUpKi, intakeconstants.slapdownUpKd);
        slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
        slapdownPID.enableContinuousInput(0, 2 * Math.PI);
        count = 0;
    }

    public void slapdowntoggle(){
        if (Math.abs(slapdownencoder.getPosition() - intakeconstants.slapdownUpAngleRad) < intakeconstants.slapdownToleranceRad) {
            slapdowndown();
        } else {
            slapdownup();
        }
    }

    public boolean slapdownattarget(){ //not used for now maybe will come in hand later trust
        return slapdownPID.atSetpoint();
    }
}