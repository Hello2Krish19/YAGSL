package swervelib.parser.json;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.function.Supplier;
import swervelib.parser.deserializer.ReflectionsManager.AbsoluteEncoder;
import swervelib.parser.deserializer.ReflectionsManager.MotorControllers;
import swervelib.parser.json.SwerveDriveJson.GyroAxis;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * SmartIO Channel.
   */
  public int    channel = 0;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus  = "";

  /**
   * Get the DC motor from the motor type.
   *
   * @param motorType Motor type.
   * @return {@link DCMotor}
   */
  public static DCMotor getDCMotor(String motorType)
  {
    switch (motorType)
    {
      case "neo2":
      case "neo":
        return DCMotor.getNEO(1);
      case "neo550":
        return DCMotor.getNeo550(1);
      case "vortex":
        return DCMotor.getNeoVortex(1);
      case "minion":
        return DCMotor.getMinion(1);
      case "krakenx44":
        return DCMotor.getKrakenX44(1);
      case "krakenx60":
        return DCMotor.getKrakenX60(1);
      case "pulsar":
        return new DCMotor(12, 3.1, 189, 1, 7500, 1);
      default:
        throw new IllegalArgumentException("Invalid motor type: " + motorType);
    }
  }

  public Supplier<Angle> getEncoderSupplier(Object azimuthVendorMotorController)
  {
    switch (getVendor(VENDOR.UNKNOWN))
    {
      case CTRE ->
      {
        return getCTREEncoder().getAbsolutePosition().asSupplier();
      }
      case REV ->
      {
        return () -> Rotations.of(getREVEncoder(azimuthVendorMotorController).getPosition());
      }
      case ANDYMARK ->
      {
        throw new UnsupportedOperationException("AndyMark hex bore encoder are not yet supported.");
      }
      case REDUX ->
      {
//        return getReduxEncoder()
        throw new UnsupportedOperationException("Redux encoder are not yet supported.");
      }
      case SMARTIO ->
      {
        Object encoder = getSmartIOEncoder();
        if (encoder instanceof DutyCycleEncoder)
        {return () -> Rotations.of(((DutyCycleEncoder) encoder).get());}
        if (encoder instanceof AnalogEncoder)
        {return () -> Rotations.of(((AnalogEncoder) encoder).get());}
      }
    }
    throw new IllegalArgumentException("Invalid encoder type: " + type);
  }

  public Supplier<Angle> getGyroSupplier(GyroAxis axis)
  {
    switch (getVendor(VENDOR.UNKNOWN))
    {
      case CTRE ->
      {
        switch (axis)
        {
          case YAW -> getCTREGyro().getYaw().asSupplier();
          case PITCH -> getCTREGyro().getPitch().asSupplier();
          case ROLL -> getCTREGyro().getRoll().asSupplier();
        }
      }
//      case REDUX -> {
//        switch (axis){
//
//        }
//      }
      case STUDICA ->
      {
        if (getStudicaGyro() instanceof Navx)
        {
          switch (axis)
          {
            case YAW ->
            {
              return () -> Degrees.of(((Navx) getStudicaGyro()).getYaw());
            }
            case PITCH ->
            {
              return () -> Degrees.of(((Navx) getStudicaGyro()).getPitch());
            }
            case ROLL ->
            {
              return () -> Degrees.of(((Navx) getStudicaGyro()).getRoll());
            }
          }
        }
      }
      case LIMELIGHT ->
      {
        throw new UnsupportedOperationException("Limelight gyro are not yet supported.");
      }
    }
    throw new IllegalArgumentException("Invalid gyro type: " + type);
  }

  public enum VENDOR
  {CTRE, REV, THRIFTYBOT, ANDYMARK, REDUX, STUDICA, SMARTIO, LIMELIGHT, UNKNOWN}

  /**
   * Get the vendor of the device.
   *
   * @return Vendor of the device.
   */
  public VENDOR getVendor(VENDOR attachedType)
  {
    if (type.contains("_"))
    {
      String[] vendorData           = type.split("_");
      String   vendorType           = vendorData[0];
      String   vendorConnectionType = vendorData[1];
      switch (vendorType)
      {
        case "systemcore":
          return VENDOR.LIMELIGHT;
        case "navx":
        case "navx2":
        case "navx3":
          return VENDOR.STUDICA;
        case "talonfx":
        case "talonfxs":
        case "cancoder":
        case "pigeon2":
          return VENDOR.CTRE;
        case "sparkmax":
        case "sparkflex":
          return VENDOR.REV;
        case "revthroughbore":
          switch (vendorConnectionType)
          {
            case "attached": return attachedType;
            case "dio": return VENDOR.SMARTIO;
          }
        case "nova":
          return VENDOR.THRIFTYBOT;
        case "andymarkhexbore":
          switch (vendorConnectionType)
          {
            case "attached": return attachedType;
            case "dio":
            case "analog": return VENDOR.SMARTIO;
            case "can": return VENDOR.ANDYMARK;
          }
        case "canandgyro": return VENDOR.REDUX;
        case "canandmag":
          switch (vendorConnectionType)
          {
            case "attached": return attachedType;
            case "dio": return VENDOR.SMARTIO;
            case "can": return VENDOR.REDUX;
          }
        case "srxmag":
          switch (vendorConnectionType)
          {
            case "attached": return attachedType;
            case "analog": return VENDOR.SMARTIO;
          }
        case "thrifty":
          switch (vendorConnectionType)
          {
            case "attached": return attachedType;
            case "analog": return VENDOR.SMARTIO;
          }
      }
    }
    return VENDOR.UNKNOWN;
  }

  public Object getStudicaGyro()
  {
    String[] vendorData           = type.split("_");
    String   vendorType           = vendorData[0];
    String   vendorConnectionType = vendorData[1];
    switch (vendorConnectionType)
    {
      case "can":
        return new Navx(id);
      default:
        throw new IllegalArgumentException("Invalid gyro connection type: " + vendorType);
    }

  }

  public Pigeon2 getCTREGyro()
  {
    return new Pigeon2(id);
  }

//  public Canandgyro getReduxGyro()
//  {
//    String[] vendorData           = type.split("_");
//    String   vendorType           = vendorData[0];
//    String   vendorConnectionType = vendorData[1];
//    switch (vendorType)
//    {
//      case "canandgyro": return new Canandgyro(id);
//      default: throw new IllegalArgumentException("Invalid encoder type: " + vendorType);
//    }
//  }

//  public Canandmag getReduxEncoder()
//  {
//    String[] vendorData           = type.split("_");
//    String   vendorType           = vendorData[0];
//    String   vendorConnectionType = vendorData[1];
//    switch (vendorType)
//    {
//      case "canandmag": return new Canandmag(id);
//      default: throw new IllegalArgumentException("Invalid encoder type: " + vendorType);
//    }
//  }

  /**
   * Get the Absolute Encoder Supplier and Vendor Absolute Encoder Object.
   *
   * @return Pair of {@link Supplier} and Vendor Absolute Encoder {@link Object}
   */
  public Pair<Supplier<Angle>, Object> getAbsoluteEncoder(MotorControllers angleMotorVendor,
                                                          SmartMotorController angleMotorController)
  {
    String[] vendorData           = type.split("_");
    String   vendorType           = vendorData[0];
    String   vendorConnectionType = vendorData[1];
    switch (vendorConnectionType)
    {
      case "attached":
        switch (vendorType)
        {
          case "canandmag": return angleMotorVendor.getAbsoluteEncoder("canandmag", angleMotorController);
          case "revthroughbore": return angleMotorVendor.getAbsoluteEncoder("revthroughbore", angleMotorController);
          case "srxmag": return angleMotorVendor.getAbsoluteEncoder("srxmag", angleMotorController);

          default: throw new IllegalArgumentException("Invalid encoder type: " + vendorType);
        }
      default:
        switch (vendorType)
        {
          case "cancoder": return AbsoluteEncoder.CANCODER.getAbsoluteEncoder(id, canbus);

          default: throw new IllegalArgumentException("Invalid encoder type: " + vendorType);
        }
    }
  }

  /**
   * Get the Spark encoder.
   *
   * @param vendorMotorController {@link SparkMax} or {@link SparkFlex} vendor motor controller
   * @return {@link SparkAbsoluteEncoder}
   */
  public SparkAbsoluteEncoder getREVEncoder(Object vendorMotorController)
  {
    if (vendorMotorController instanceof SparkBase) {return ((SparkBase) vendorMotorController).getAbsoluteEncoder();}
    throw new IllegalArgumentException(
        "Invalid vendor motor controller type: " + vendorMotorController.getClass().getSimpleName());
  }

  /**
   * SmartIO Encoder.
   *
   * @return {@link DutyCycleEncoder}
   */
  public Object getSmartIOEncoder()
  {
    String[] vendorData           = type.split("_");
    String   vendorType           = vendorData[0];
    String   vendorConnectionType = vendorData[1];
    switch (vendorConnectionType)
    {
      case "dio": return new DutyCycleEncoder(id);
      case "analog": return new AnalogEncoder(channel);
    }
    throw new IllegalArgumentException("Invalid encoder connection type: " + vendorConnectionType);
  }

  /**
   * AndyMark Hex Bore Encoder.
   *
   * @return Object
   */
  public Object getAndyMarkEncoder()
  {
    throw new UnsupportedOperationException("AndyMark hex bore encoder are not yet supported.");
  }

  /**
   * Get the {@link SmartMotorController} from the {@link DeviceJson} when given the
   * {@link SmartMotorControllerConfig}.
   *
   * @param config                {@link SmartMotorControllerConfig} to apply when creating
   *                              {@link SmartMotorController}.
   * @param vendorMotorController Vendor motor controller.
   * @return {@link SmartMotorController}
   */
  public SmartMotorController getMotorController(SmartMotorControllerConfig config, Object vendorMotorController)
  {
    String[] subtypes            = type.split("_");
    String   motorControllerType = subtypes[0].toLowerCase();
    String   motorType           = subtypes[1].toLowerCase();
    DCMotor  motor               = getDCMotor(motorType);
    if (vendorMotorController == null)
    {
      vendorMotorController = getVendorMotorController();
    }
    if (vendorMotorController instanceof TalonFX)
    {
      return new TalonFXWrapper((TalonFX) vendorMotorController, motor, config);
    } else if (vendorMotorController instanceof TalonFXS)
    {
      return new TalonFXSWrapper((TalonFXS) vendorMotorController, motor, config);
    } else if (vendorMotorController instanceof SparkMax)
    {
      return new SparkWrapper((SparkMax) vendorMotorController, motor, config);
    } else if (vendorMotorController instanceof SparkFlex)
    {
      return new SparkWrapper((SparkFlex) vendorMotorController, motor, config);
    } else if (vendorMotorController instanceof ThriftyNova)
    {
      return new NovaWrapper((ThriftyNova) vendorMotorController, motor, config);
    }
    throw new IllegalArgumentException("Invalid motor controller type: " + motorControllerType);
  }

  /**
   * Get the vendor motor controller.
   *
   * @return Vendor motor controller.
   */
  public Object getVendorMotorController()
  {
    String[] subtypes            = type.split("_");
    String   motorControllerType = subtypes[0].toLowerCase();
    String   motorType           = subtypes[1].toLowerCase();
    switch (motorControllerType)
    {
      case "talonfx":
        return new TalonFX(id, new CANBus(canbus));
      case "talonfxs":
        return new TalonFXS(id, new CANBus(canbus));
      case "sparkmax":
        return new SparkMax(id, MotorType.kBrushless);
      case "sparkflex":
        return new SparkFlex(id, MotorType.kBrushless);
      case "nova":
        switch (motorType)
        {
          case "neo":
          case "neo2":
          case "neo550":
          case "vortex":
            return new ThriftyNova(id, ThriftyNova.MotorType.NEO);
          case "minion":
            return new ThriftyNova(id, ThriftyNova.MotorType.MINION);
        }
      default:
        throw new IllegalArgumentException("Invalid motor controller type: " + motorControllerType);
    }
  }


}
