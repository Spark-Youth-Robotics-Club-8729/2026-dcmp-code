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
import edu.wpi.first.wpilibj.DigitalInput;
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

import edu.wpi.first.math.MathUtil;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX  intakemotor;
    private final SparkMax slapdownmotor;
    private final AbsoluteEncoder slapdownencoder;


    private final Debouncer slapdownSettleDebouncer = new Debouncer(0.15, DebounceType.kRising);
    // Detect bumper contact (stall) and stop driving the slapdown PID when we hit a hard stop

    // detect limit switch state (use limitSwitch.get())
    DigitalInput limitSwitch = new DigitalInput(intakeconstants.LIMIT_PORT);

    // GO TO HUB AND SET IT TO THE INTIAL ONE BEFORE MATCH
    private double absoluteOffset = intakeconstants.absoluteOffset;

    private final Debouncer slapdownStallDebouncer =
            new Debouncer(0.15, DebounceType.kRising);

    private double jitterAnchor = 0;
    private PIDController slapdownPID;
    private double slapdowntarget;
    public double commandVoltage;
    public IntakeSubsystem() {
        intakemotor = new TalonFX(intakeconstants.kRollerID);
        slapdownmotor = new SparkMax(intakeconstants.kSlapdownID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        //set the motor to coast
        config.idleMode(SparkMaxConfig.IdleMode.kCoast);

        config.absoluteEncoder
            .positionConversionFactor((2.0 * Math.PI))
            .velocityConversionFactor((2.0 * Math.PI) / 60)
            .inverted(true);

        slapdownmotor.configure(config,ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        slapdownencoder = slapdownmotor.getAbsoluteEncoder();
        slapdownPID = new PIDController(intakeconstants.slapdownUpKp, 0, 0); //readd derivative and tune later

        slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
        slapdowntarget = currentPosition();
        //slapdownPID.enableContinuousInput(0, 2 * Math.PI);


    }
    
    //private final Debouncer limitDebouncer = new Debouncer(0.05, DebounceType.kBoth);
    private final Alert  motordisconnect = new Alert("Intake motor disconnected", Alert.AlertType.kWarning);

    public void periodic() {
        //System.out.println("current angle "+slapdownencoder.getPosition());
        System.out.println("current position "+currentPosition()+"offset"+absoluteOffset);
        System.out.println("limit switch state "+!limitSwitch.get());

    
        
        //is true when detects magnet (resets encoder position when reaching the top)
        if (!limitSwitch.get()) {
            resetEncoder();
        }


        //gets current position based on the absolute offset
        double currentPosition = currentPosition();
        double output = slapdownPID.calculate(currentPosition, slapdowntarget)*intakeconstants.slapdowngearratio;

        // Stall detection: if motor is stalled (high voltage, high current, low velocity), set output to 0
        boolean slapdownStalled = slapdownStallDebouncer.calculate(
            Math.abs(output) >= intakeconstants.slapdownStallAppliedVolts
                && Math.abs(slapdownencoder.getVelocity()) <= intakeconstants.slapdownStallVelocityRadPerSec);
        
        if (slapdownStalled) {
            output = 0;
        }
        
        slapdownmotor.setVoltage(output);

        // use for logging and elastic in the future
    }

    //reset zero offset when at the top
    public void resetEncoder() {
        absoluteOffset = slapdownencoder.getPosition();
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
    }

    public void slapdownup(){
        //sets the goal and the new PID
        slapdowntarget = intakeconstants.slapdownUpAngleRad;
        slapdownPID = new PIDController(intakeconstants.slapdownUpKp, intakeconstants.slapdownUpKi, intakeconstants.slapdownUpKd);
        slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
        //slapdownPID.enableContinuousInput(0, 2 * Math.PI);
    }

    // jittering
    public void slapdownjitterUp(double angle){
        slapdowntarget = angle - intakeconstants.jitterRangeRad;
        
        slapdownPID = new PIDController(
            intakeconstants.slapdownJUpKp, 
            intakeconstants.slapdownJUpKi, 
            intakeconstants.slapdownJUpKd
        );
        slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
        slapdownPID.enableContinuousInput(0, 2 * Math.PI);
    }
    
    public void slapdownjitterDown(double angle){
        // Return exactly to where we started (get the jitterAnchor when you press the button intially)
        if (angle > 1.45) {
            slapdowntarget = angle;
        } else {
            slapdowntarget = angle + intakeconstants.jitterRangeRad;
        }
        
        slapdownPID = new PIDController(
            intakeconstants.slapdownJDownKp, 
            intakeconstants.slapdownJDownKi, 
            intakeconstants.slapdownJDownpKd
        );
        slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
        slapdownPID.enableContinuousInput(0, 2 * Math.PI);
    }



    // gets current encoder position based on absolute offset and wraps 
    public double currentPosition () {
        double currentPosition = slapdownencoder.getPosition() - absoluteOffset;

        // wraps around to be within 0 and 2pi .. changed to -pi to pi to avoid issues from jumping between 0 and 2pi (TEST FIRST)
        return MathUtil.inputModulus(currentPosition, -Math.PI, Math.PI);
    }

    // toggle function of slapdown up or down based on current position
    public void slapdowntoggle() {
        double currentPosition = currentPosition();

        // true is when the slapdown is up ()
        boolean slapState = true;

        if((0.4 < currentPosition && currentPosition < 3.0)) {
            slapState = false;
        } else {
            slapState = true;
        }

        System.out.println("slap state (true = top): " + slapState);

        // based on current slapState it will run the slapdown up or down
        if (slapState) {
            slapdownup();
        } else {
            slapdowndown();
        }
    }

    public boolean slapdownattarget(){ //not used for now maybe will come in hand later trust
        return slapdownPID.atSetpoint();
    }
}
