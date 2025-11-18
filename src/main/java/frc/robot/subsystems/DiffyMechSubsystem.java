package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.DifferentialMechanismConfig;
import yams.mechanisms.positional.DifferentialMechanism;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

public class DiffyMechSubsystem extends SubsystemBase{
  private final SparkMax motorOne = new SparkMax(1, SparkLowLevel.MotorType.kBrushed);
  private final SmartMotorControllerConfig oneConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(16, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      //.withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.75)))
      //.withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
      .withTelemetry("motorOne", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(133))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
  private final SmartMotorController SMCOne = new SparkWrapper(motorOne, DCMotor.getNEO(1), oneConfig);
    private final SparkMax                   motorTwo  = new SparkMax(2, SparkLowLevel.MotorType.kBrushed);
    private final SmartMotorControllerConfig twoConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(16, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    //.withSoftLimit(Degrees.of(-30), Degrees.of(100))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.75)))
    //.withExternalEncoder(armMotor.getAbsoluteEncoder())
    .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
    .withTelemetry("motorTwo", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
    .withStatorCurrentLimit(Amps.of(133))
    .withMotorInverted(false)
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
    .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final SmartMotorController SMCTwo = new SparkWrapper(motorTwo, DCMotor.getNEO(1), twoConfig);
    private final DifferentialMechanismConfig config = new DifferentialMechanismConfig(SMCOne, SMCTwo)
            .withStartingPosition(Degrees.of(90), Degrees.of(0))
            .withTelemetry("DiffyMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    private final DifferentialMechanism diffy     = new DifferentialMechanism(config);
  
    public DiffyMechSubsystem()
    {
    }
  
    public Command setAngle(Angle tilt, Angle twist) {
      return diffy.setPosition(tilt, twist);
    }
  
    public Command set(double tilt, double twist) {
      return diffy.set(tilt, twist);
    }
  
    public void periodic()
    {
      diffy.updateTelemetry();
    }
  
    public void simulationPeriodic()
    {
      diffy.simIterate();
    }
}