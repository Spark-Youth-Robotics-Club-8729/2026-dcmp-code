// package frc.robot.subsystems;

// import static frc.robot.Constants.intakeconstants;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ResetMode;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.intakeconstants;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.REVLibError;
// import com.revrobotics.spark.SparkBase;

// public class IntakeSubsystem extends SubsystemBase{
//     public static int count = 0;
//     private final TalonFX  intakemotor;
//     private final SparkMax slapdownmotor;
//     private final AbsoluteEncoder slapdownencoder;
//     private final PIDController slapdownPID;
//     private double slapdowntarget;
//     public double commandVoltage;
//     public IntakeSubsystem() {
//         intakemotor = new TalonFX(intakeconstants.kRollerID);
//         slapdownmotor = new SparkMax(intakeconstants.kSlapdownID, MotorType.kBrushless);
//         SparkMaxConfig config = new SparkMaxConfig();

//         config.absoluteEncoder
//             .positionConversionFactor(2*Math.PI)
//             .velocityConversionFactor(2*Math.PI)
//             .inverted(true);

//         slapdownmotor.configure(config,ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//         slapdownencoder = slapdownmotor.getAbsoluteEncoder();
//         slapdownPID = new PIDController(intakeconstants.slapdownUpKp, 0, 0); //readd derivative and tune later

//         slapdownPID.setTolerance(intakeconstants.slapdownToleranceRad);
//         slapdowntarget = slapdownencoder.getPosition();
//         slapdownPID.enableContinuousInput(0, 2 * Math.PI);

//     }

    
//     private final Alert  motordisconnect = new Alert("Intake motor disconnected", Alert.AlertType.kWarning);

//     public void periodic() {
        
//         double output = slapdownPID.calculate(slapdownencoder.getPosition(),slapdowntarget)*intakeconstants.slapdowngearratio;
//         output = Math.max(-6.0, Math.min(6.0, output));
//         if(slapdownPID.atSetpoint()){
//             slapdownmotor.setVoltage(0);
//         }
//         else{
//             slapdownmotor.setVoltage(output);
//         }

//         //System.out.println("current position" + slapdownencoder.getPosition());
//         //System.out.println("target" + slapdowntarget);

//         // use for logging and elastic in the future
//     }
//     public void setVoltage(double volts) {
//         intakemotor.setVoltage(volts);
//     }
//     public double getvoltage(){
//         return -intakemotor.get();
//     }
//     public void intake(){
//         setVoltage(intakeconstants.rollerIntakeVolts);
        
//     }

//     public void outtake(){
//         setVoltage(intakeconstants.rollerOuttakeVolts);
//     }

//     public void stopintake(){
//         setVoltage(0);
//     }


//     //slapdown 
//     public void slapdowndown(){
//         slapdowntarget = intakeconstants.slapdownDownAngleRad;
//         count +=1;
//     }
//     public void slapdownup(){
//         slapdowntarget = intakeconstants.slapdownUpAngleRad;
//         count+=1;
//     }
//     public boolean slapdownattarget(){ //not used for now maybe will come in hand later trust
//         return slapdownPID.atSetpoint();
//     }



// }
