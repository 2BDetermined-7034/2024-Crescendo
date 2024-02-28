package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
	protected TalonFX lowerMotor, upperMotor;
	protected PositionVoltage positionVoltage;
	protected double targetPosition;
	boolean manualOverrideActive;
	private Follower lowerMotorFollower;

	public ClimbSubsystem() {
		lowerMotor = new TalonFX(Constants.Climb.lowerKrakenID, Constants.CANBUS_NAME);
		upperMotor = new TalonFX(Constants.Climb.upperKrakenID, Constants.CANBUS_NAME);

		lowerMotor.setNeutralMode(NeutralModeValue.Brake);
		upperMotor.setNeutralMode(NeutralModeValue.Brake);

		Slot0Configs motorConfig = new Slot0Configs();
		motorConfig.kP = 1.0;
		motorConfig.kI = 0.0;
		motorConfig.kD = 0.0;
		motorConfig.kS = 0.2;

		upperMotor.getConfigurator().apply(motorConfig);

		lowerMotorFollower = new Follower(Constants.Climb.upperKrakenID, false);

		positionVoltage = new PositionVoltage(0);

		upperMotor.setPosition(0.0);
		lowerMotor.setPosition(0.0);
	}

	@Override
	public void periodic() {
		lowerMotor.setControl(lowerMotorFollower);
	}

	/**
	 *
	 * @param position Target position for the climb in motor revolutions
	 */
	public void setPosition(double position) {
		positionVoltage.Position = position;
		upperMotor.setControl(positionVoltage);
	}

	/**
	 *
	 * @param speed Sets the raw velocity of the motors
	 */
	public void setOverrideVelocity(double speed) {
		upperMotor.set(speed);
		manualOverrideActive = speed != 0;
	}

	/**
	 * Returns motor current from upper (higher geared) kraken
	 * @return
	 */
	public double getMotorCurrent() {
		return upperMotor.getStatorCurrent().getValue();
	}

	public boolean atCurrentLimit() {
		return getMotorCurrent() > Constants.Climb.currentLimit;
	}
	public boolean useClosedLoop(){
		return !atCurrentLimit() && !manualOverrideActive;
	}

	public double getPosition(){
		return upperMotor.getPosition().getValue();
	}

	public void coast(){
		upperMotor.setControl(new CoastOut());
	}
}
