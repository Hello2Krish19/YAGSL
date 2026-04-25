package swervelib.parser.deserializer.reflections;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Reflection class for {@link com.revrobotics.spark.SparkBase}s and other REV devices.
 */
public class REVDevices
{

  /**
   * Motor controller types.
   */
  public enum MotorControllerType
  {
    /**
     * {@link com.revrobotics.spark.SparkFlex}
     */
    SPARKFLEX,
    /**
     * {@link com.revrobotics.spark.SparkMax}
     */
    SPARKMAX
  }

  public enum AbsoluteEncoder
  {
    /**
     * {@link com.revrobotics.encoder.SplineEncoder}
     */
    SPLINEENCODER
  }


  /**
   * Get the {@link com.revrobotics.spark.SparkBase} as a {@link SmartMotorController}.
   *
   * @param canid               CAN ID of the {@link com.revrobotics.spark.SparkBase}
   * @param canbus              CAN bus name of the {@link com.revrobotics.spark.SparkBase}
   * @param config              {@link SmartMotorControllerConfig} to apply to the {@link SmartMotorController}
   * @param motor               {@link DCMotor} to use with the {@link SmartMotorController}
   * @param motorControllerType Motor controller type.
   * @return {@link SmartMotorController}
   */
  public static SmartMotorController getMotorController(int canid, String canbus, SmartMotorControllerConfig config,
                                                        DCMotor motor, String motorControllerType)
  {
    // Will throw an error if invalid motor controller type is given.
    var       motorType       = MotorControllerType.valueOf(motorControllerType.toUpperCase());
    SparkBase motorController = null;
    switch (motorType)
    {
      case SPARKFLEX ->
      {
        motorController = new SparkFlex(canid, MotorType.kBrushless);
      }
      case SPARKMAX ->
      {
        motorController = new SparkMax(canid, MotorType.kBrushless);
      }
    }
    return new SparkWrapper(motorController, motor, config);
  }

  /**
   * Get the {@link Angle} {@link Supplier} and the encoder object.
   *
   * @param canid  CAN ID of the encoder.
   * @param canbus CAN bus name for the encoder.
   * @return {@link Pair} of {@link Supplier} and {@link Object}
   * @implNote {@link Angle} is in the range of [0, 1) by default.
   */
  public static Pair<Supplier<Angle>, Object> getAbsoluteEncoder(int canid, String canbus)
  {
    var encoder = new SplineEncoder(canid);
    return Pair.of(() -> Rotations.of(encoder.getAngle()), encoder);
  }

  public static Pair<Supplier<Angle>, Object> getAttachedAbsoluteEncoder(String attachType, Object motorController)
  {
    SparkAbsoluteEncoder encoder;
    if (motorController instanceof SparkMax)
    {
      encoder = ((SparkMax) motorController).getAbsoluteEncoder();

    } else if (motorController instanceof SparkFlex)
    {
      encoder = ((SparkFlex) motorController).getAbsoluteEncoder();
    } else
    {
      throw new IllegalArgumentException(
          "Invalid motor controller type: " + motorController.getClass().getSimpleName());
    }
    switch (attachType)
    {
      case "srxmag":
      case "canandmag":
      case "revthrouhgbore":
        return Pair.of(() -> Rotations.of(encoder.getPosition()), encoder);
      default:
        throw new IllegalArgumentException("Invalid encoder type: " + attachType);
    }
  }
}
