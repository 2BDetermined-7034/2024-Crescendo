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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
	private final TalonFX launchTalon;
	private final TalonFX angleTalon;
	private final CANSparkMax lowGearNeo;
	private final CANSparkMax highGearNeo;
	private final PositionVoltage anglePositionController;
	private final VelocityVoltage launchVelocityController;
	private double shooterPercent;


	private final TrapezoidProfile angleProfile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(20, 40)
	);
	private TrapezoidProfile.State angleSetpoint = new TrapezoidProfile.State(); //Setpoint

	/**
	 * periodically assigned to the setpoint of the positionvoltage controller
	 */
	public double angleMotorPosition = 0; //Goal state

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

		angleTalon.setInverted(false);
		lowGearNeo.setInverted(true);
		highGearNeo.setInverted(true);

		launchTalon.setNeutralMode(NeutralModeValue.Coast);
		angleTalon.setNeutralMode(NeutralModeValue.Brake);
		lowGearNeo.setIdleMode(CANSparkBase.IdleMode.kBrake);
		highGearNeo.setIdleMode(CANSparkBase.IdleMode.kBrake);



		Slot0Configs slot0Configs;

		slot0Configs = new Slot0Configs();
		slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
		slot0Configs.kP = 3;
		slot0Configs.kI = 10.1;
		slot0Configs.kD = 0;
		angleTalon.getConfigurator().apply(slot0Configs, 0.050);

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


		angleTalon.setPosition(angleDegreesToRotations(Shooter.angleBackHardstop));

		anglePositionController = new PositionVoltage(angleDegreesToRotations(Shooter.angleBackHardstop));
		launchVelocityController = new VelocityVoltage(0);

		//init encoder to 0 position on boot
	}

	public void periodic() {
		// periodic, update the profile setpoint for 20 ms loop time
		angleSetpoint = angleProfile.calculate(0.040, angleSetpoint, new TrapezoidProfile.State(angleMotorPosition, 0));
		// apply the setpoint to the control request
		anglePositionController.Position = angleSetpoint.position;
		anglePositionController.Velocity = angleSetpoint.velocity;
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
		SmartDashboard.putNumber("Angle Motor Acceleration", angleTalon.getAcceleration().getValue());
		SmartDashboard.putNumber("Angle Motor Velocity", angleTalon.getVelocity().getValue());
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
		return rotations * Shooter.angleGearRatio * (360) + Shooter.angleBackHardstop;
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
		return (degrees - Shooter.angleBackHardstop) / (Shooter.angleGearRatio * 360);
	}


	/**
	 * Sets the shooter angle to the shooter relative degree angle
	 * @param degrees position in degrees
	 */
	public void setAngleTalonPositionDegrees(double degrees){
		angleMotorPosition = angleDegreesToRotations(MathUtil.clamp(degrees, Shooter.angleFrontHardstop, Shooter.angleBackHardstop));
	}

	/**
	 * Dont make this positive
	 * @param rotations input
	 */
	private void setAngleTalonPositionRotations(double rotations) {
		angleMotorPosition = rotations;
	}

	/**
	 * sets angle kraken speed [-1, 1]
	 * @param speed percent speed of angle kraken
	 */
	private void setAngleTalon(double speed) {
		angleTalon.set(speed);
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


}
