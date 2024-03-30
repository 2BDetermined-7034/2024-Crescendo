package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX launchTalon;
    private final TalonFX angleTalon;
    private final CANSparkMax lowGearNeo;
    private final CANSparkMax highGearNeo;
    private final CANcoder angleCANCoder;
    private final PositionVoltage anglePositionController;
    private final VelocityVoltage launchVelocityController;
    private final PositionVoltage backToHardstopController;

    private double shooterPercent;


    private final TrapezoidProfile angleProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(20, 40)
    );
    private TrapezoidProfile.State angleSetpoint = new TrapezoidProfile.State(); //Setpoint

    /**
     * periodically assigned to the setpoint of the positionvoltage controller
     */
    public double angleMotorSetpoint = Shooter.angleBackHardstop; //Goal state

    /**
     * periodically assigned to as the setpoint of the velocityVoltageController
     */
    public double launchMotorVelocity = 0;

    private final String smdbLoggingString = "Shooter/";

    /**
     * Initialize the robot's {@link ShooterSubsystem},
     * which includes 2 (angle and launch) Krakens, and two (low gear and high gear) Neo 550s.
     */
    public ShooterSubsystem() {
        SmartDashboard.putNumber("Set Shooter Angle", 0);
        this.launchTalon = new TalonFX(Shooter.launchKrakenID);
        this.angleTalon = new TalonFX(Shooter.angleFalconID);
        this.lowGearNeo = new CANSparkMax(Shooter.neo550LowGearID, CANSparkLowLevel.MotorType.kBrushless);
        this.highGearNeo = new CANSparkMax(Shooter.neo550HighGearID, CANSparkLowLevel.MotorType.kBrushless);
        this.angleCANCoder = new CANcoder(Shooter.angleCANCoderID);

        angleTalon.setInverted(true);
        lowGearNeo.setInverted(true);
        highGearNeo.setInverted(true);

        launchTalon.setNeutralMode(NeutralModeValue.Coast);
        angleTalon.setNeutralMode(NeutralModeValue.Brake);
        lowGearNeo.setIdleMode(CANSparkBase.IdleMode.kBrake);
        highGearNeo.setIdleMode(CANSparkBase.IdleMode.kBrake);

        // Configs

        CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
        angleCANCoderConfig.MagnetSensor.MagnetOffset = Shooter.angleCanCoderZero;

        // PID values when driving to an angle

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
        slot0Configs.kP = 3;
        slot0Configs.kI = 10.1;
        slot0Configs.kD = 0;

        // PID values when driving to the hardstop

        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.kS = 0;
        slot1Configs.kP = 0;
        slot1Configs.kI = 0;
        slot1Configs.kD = 0;

        TalonFXConfiguration angleTalonConfig = new TalonFXConfiguration();
        angleTalonConfig.Feedback.SensorToMechanismRatio = 1d / Shooter.angleCancoderGearRatio;
        angleTalonConfig.Feedback.FeedbackRemoteSensorID = angleCANCoder.getDeviceID();
        angleTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        angleTalonConfig.Slot0 = slot0Configs;
        angleTalonConfig.Slot1 = slot1Configs;
        angleTalon.getConfigurator().apply(angleTalonConfig, 0.050);

//		slot0Configs = new Slot0Configs();
//		//slot0Configs.kS = 0.24;
//		slot0Configs.kP = 1;
//		slot0Configs.kI = 0;
//		slot0Configs.kD = 0;
//
//		shooterPercent = 0;
//
//		angleTalon.getConfigurator().apply(slot0Configs);

        var launchMotorPID = new Slot0Configs();
        launchMotorPID.kV = 0.12;
        launchMotorPID.kP = 1;
        launchMotorPID.kI = 0.00;
        launchMotorPID.kD = 0.0;
        launchMotorPID.kS = 0.24;
        launchTalon.getConfigurator().apply(launchMotorPID, 0.050);

        anglePositionController = new PositionVoltage(angleDegreesToRotations(Shooter.angleBackHardstop));
        launchVelocityController = new VelocityVoltage(0);
        backToHardstopController = new PositionVoltage(Shooter.angleBackHardstop, 0.0, false, 0.0, 1, false, false, false);

        //init encoder to 0 position on boot
    }

    public void periodic() {
        // Will run the launch Kraken to coast out when given a stop command (vel = 0).
        // This prevents harshly braking the flywheels causing excess heat.

        if(launchMotorVelocity != 0){
            launchVelocityController.Velocity = MathUtil.clamp(launchMotorVelocity, -80, 80);
            launchTalon.setControl(launchVelocityController);
        } else {
            launchTalon.setControl(new NeutralOut());
        }

        if(angleMotorSetpoint == Shooter.angleBackHardstop){
            angleTalon.setControl(backToHardstopController);
        } else {
            // periodic, update the profile setpoint for 20 ms loop time
            angleSetpoint = angleProfile.calculate(0.040, angleSetpoint, new TrapezoidProfile.State(angleMotorSetpoint, 0));
            // apply the setpoint to the control request
            anglePositionController.Position = angleSetpoint.position;
            anglePositionController.Velocity = angleSetpoint.velocity;
            angleTalon.setControl(anglePositionController);
        }

        logging();
    }

    private void logging() {
        SmartDashboard.putNumber("Shooter/Angle Position Degrees", getAnglePositionDegrees());
        SmartDashboard.putNumber("Shooter/Angle Setpoint Degrees", angleMotorSetpoint);
        SmartDashboard.putNumber("Shooter/Angle Motor Current", angleTalon.getTorqueCurrent().getValue());
        SmartDashboard.putNumber("Shooter/Angle Motor Acceleration", angleTalon.getAcceleration().getValue());
        SmartDashboard.putNumber("Shooter/Angle Motor Velocity", angleTalon.getVelocity().getValue());
        SmartDashboard.putNumber("Shooter/Launch Motor Velocity", launchTalon.getVelocity().getValue());
        SmartDashboard.putNumber("Shooter/Launch Motor Current", launchTalon.getTorqueCurrent().getValue());
    }

    /**
     * Sets the velocity setpoint for the launch Falcon.
     * @param targetVelocity Target velocity for the launch Falcon (in RPS).
     */
    public void setLaunchTalon(double targetVelocity){
        launchMotorVelocity = targetVelocity;
    }

    /**
     * Gets the current velocity of the launch Falcon.
     * @return Current velocity of the launch Falcon (in RPS).
     */
    public double getLaunchMotorVelocity(){
        return launchTalon.getVelocity().getValue();
    }

    public double getAngleMotorVelocity() {
        return angleTalon.getVelocity().getValue();
    }


    /**
     * Gets the current rotations of the angle kraken in motor relative rotation units
     * @return rotation position of angle kraken
     */
    public double getAnglePositionRotations() {
        return angleTalon.getPosition().getValue();
    }

    public double getAnglePositionDegrees() {
        return angleTalon.getPosition().getValue();
    }

    /**
     * Converts from rotations of the angle Kraken to degrees from the horizon.
     * Positive indicates angling the shooter upwards.
     * Zero is the angle with the shooter along positive x robot relative coordinates
     * @param rotations Rotations of the angle Kraken.
     * @return Degrees (relative to the horizon).
     */
    public double angleRotationsToDegrees(double rotations){
        return rotations;
    }

    public void runLaunchPercent(double percent){
        shooterPercent = percent;
    }

    /**
     * Converts from degrees relative to the horizon to rotations of the angle Kraken relative to the hardstop.
     * Positive rotations indicates
     * @param degrees input
     * @return rotations
     */
    public double angleDegreesToRotations(double degrees) {
        return degrees;
    }


    /**
     * Sets the shooter angle to the shooter relative degree angle
     * @param degrees position in degrees
     */
    public void setAngleTalonPositionDegrees(double degrees) {
        angleMotorSetpoint = degrees;
    }

    /**
     * Dont make this positive
     * @param rotations input
     */
    private void setAngleTalonPositionRotations(double rotations) {
        angleMotorSetpoint = rotations;
    }

    /**
     * Sets the indexing neo550 speeds on percent from [-1, 1]
     * @param speed percent speed of indexing neos
     */
    public void setNeoSpeeds(double speed) {
        lowGearNeo.set(MathUtil.clamp(speed, -1, 1));
        highGearNeo.set(MathUtil.clamp(speed, -1, 1));

    }

    public double getAngleAcceleration() {
        return angleTalon.getAcceleration().getValue();
    }

    public double getAngleVelocity() {
        return angleTalon.getVelocity().getValue();
    }

    /**
     * Conditions for launch are:
     * Launch motor at velocity,
     * Angle motor within degree tolerance,
     * Angle motor acceleration within tolerance,
     * Angle motor velocity within tolerance.
     * These conditions should prevent firing before the shooter has reached a stable configuration
     * @param angleSetpoint The degree the angle motor is driving to
     * @return If the shooter is within tolerance to shoot
     */
    public boolean withinShootingTolerances(double angleSetpoint){
        return withinShootingTolerances(angleSetpoint, Shooter.shooterVelSetpoint);
    }

    public boolean withinShootingTolerances(double angleSetpoint, double velocitySetpoint){
        return Math.abs(getLaunchMotorVelocity() - velocitySetpoint) < Constants.Shooter.shooterVelTolerance
                && Math.abs(getAnglePositionDegrees() - angleSetpoint) < 0.5
                && Math.abs(getAngleAcceleration()) < 2
                && Math.abs(getAngleVelocity()) < 2;
    }
}