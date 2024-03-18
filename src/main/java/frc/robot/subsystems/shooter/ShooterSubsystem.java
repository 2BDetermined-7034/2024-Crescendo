package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Shooter.*;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
	private final TalonFX launchTalon;
	private final TalonFX angleTalon;
	private final CANSparkMax lowGearNeo;
	private final CANSparkMax highGearNeo;
	private final PositionVoltage anglePositionController;
	private final VelocityVoltage launchVelocityController;
	private double shooterPercent;

	/**
	 * periodically assigned to the setpoint of the positionvoltage controller
	 */
	public double angleMotorPosition = 0;

	/**
	 * periodically assigned to as the setpoint of the velocityVoltageController
	 */
	public double launchMotorVelocity = 0;

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

		angleTalon.setInverted(true);
		lowGearNeo.setInverted(true);
		highGearNeo.setInverted(true);

		launchTalon.setNeutralMode(NeutralModeValue.Coast);
		angleTalon.setNeutralMode(NeutralModeValue.Brake);
		lowGearNeo.setIdleMode(CANSparkBase.IdleMode.kBrake);
		highGearNeo.setIdleMode(CANSparkBase.IdleMode.kBrake);

		Slot0Configs angleMotorPID = new Slot0Configs();
		angleMotorPID.kP = anglePIDConstants.kP;
		angleMotorPID.kI = anglePIDConstants.kI;
		angleMotorPID.kD = anglePIDConstants.kD;
		angleMotorPID.kS = angleFeedForward;

		angleTalon.getConfigurator().apply(angleMotorPID);
		shooterPercent = 0;




// robot init, set slot 0 gains
		var launchMotorPID = new Slot0Configs();
		launchMotorPID.kV = 0.12;
		launchMotorPID.kP = 0.11;
		launchMotorPID.kI = 0.48;
		launchMotorPID.kD = 0.01;
		launchTalon.getConfigurator().apply(launchMotorPID, 0.050);


		angleTalon.setPosition(angleDegreesToRotations(Shooter.angleBackHardstop));

		anglePositionController = new PositionVoltage(angleDegreesToRotations(Shooter.angleBackHardstop));
		launchVelocityController = new VelocityVoltage(0);

		//init encoder to 0 position on boot
	}

	public void periodic() {
		anglePositionController.Position = angleMotorPosition;
		angleTalon.setControl(anglePositionController);

		if(shooterPercent != 0){
			launchTalon.set(shooterPercent);
		} else if(launchMotorVelocity != 0){
			launchVelocityController.Velocity = MathUtil.clamp(launchMotorVelocity, -80, 80);
			launchTalon.setControl(launchVelocityController);
		} else {
			launchTalon.setControl(new NeutralOut());
		}

		logging();
	}

	private void logging() {
		SmartDashboard.putNumber("Angle Position Rotations", getAnglePositionRotations());
		SmartDashboard.putNumber("Launch Kraken Velocity", getLaunchMotorVelocity());
		SmartDashboard.putNumber("Angle Position Degrees", getAnglePositionDegrees());
		SmartDashboard.putNumber("Angle Setpoint", anglePositionController.Position);
		SmartDashboard.putNumber("Angle Setpoint Degrees", angleRotationsToDegrees(anglePositionController.Position));
		SmartDashboard.putNumber("Angle Supply Voltage", angleTalon.getSupplyVoltage().getValue());
		SmartDashboard.putNumber("Angle Motor Voltage", angleTalon.getMotorVoltage().getValue());
		SmartDashboard.putNumber("Launch Current", launchTalon.getTorqueCurrent().getValue());
		SmartDashboard.putNumber("Angle Motor Current", angleTalon.getTorqueCurrent().getValue());
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
	 * @return
	 */
	public double getAnglePositionRotations() {
		return angleTalon.getPosition().getValue();
	}

	public double getAnglePositionDegrees() {
		return angleRotationsToDegrees(angleTalon.getPosition().getValue());
	}

	/**
	 * Converts from rotations of the angle Kraken to degrees from the horizon.
	 * Positive indicates angling the shooter upwards.
	 * Zero is the angle with the shooter along positive x robot relative coordinates
	 * @param rotations Rotations of the angle Kraken.
	 * @return Degrees (relative to the horizon).
	 */
	public double angleRotationsToDegrees(double rotations){
		return rotations * Shooter.angleGearRatio * (360 / 1) + Shooter.angleBackHardstop;
	}

	public void runLaunchPercent(double percent){
		shooterPercent = percent;
	}

	/**
	 * Converts from degrees relative to the horizon to rotations of the angle Kraken relative to the hardstop.
	 * Positive rotations indicates
	 * @param degrees
	 * @return
	 */
	public double angleDegreesToRotations(double degrees) {
		return (degrees - Shooter.angleBackHardstop) / (Shooter.angleGearRatio * 360);
	}


	/**
	 * Sets the shooter angle to the shooter relative degree angle
	 * @param degrees
	 */
	public void setAngleTalonPositionDegrees(double degrees){
		angleMotorPosition = angleDegreesToRotations(MathUtil.clamp(degrees, Shooter.angleFrontHardstop, Shooter.angleBackHardstop));
	}

	/**
	 * Dont make this positive
	 * @param rotations
	 */
	public void setAngleTalonPositionRotations(double rotations) {
		angleMotorPosition = rotations;
	}

	/**
	 * sets angle kraken speed [-1, 1]
	 * @param speed
	 */
	public void setAngleTalon(double speed) {
		angleTalon.set(speed);
	}

	/**
	 * Sets the indexing neo550 speeds on percent from [-1, 1]
	 * @param speed
	 */
	public void setNeoSpeeds(double speed) {
		lowGearNeo.set(MathUtil.clamp(speed, -1, 1));
		highGearNeo.set(MathUtil.clamp(speed, -1, 1));

	}
}
