package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		upperMotor = new TalonFX(Constants.Climb.upperFalconID, Constants.CANBUS_NAME);

		lowerMotor.setNeutralMode(NeutralModeValue.Brake); // TODO SET TO BRAKE
		upperMotor.setNeutralMode(NeutralModeValue.Brake);

		Slot0Configs motorConfig = new Slot0Configs();
		motorConfig.kP = 1.0;
		motorConfig.kI = 0.0;
		motorConfig.kD = 0.0;
		motorConfig.kS = 0.2;

		upperMotor.getConfigurator().apply(motorConfig);

		lowerMotorFollower = new Follower(Constants.Climb.upperFalconID, true);

		positionVoltage = new PositionVoltage(0);

		upperMotor.setInverted(true);

		upperMotor.setPosition(0.0);
		lowerMotor.setPosition(0.0);
	}

	@Override
	public void periodic() {
		lowerMotor.setControl(lowerMotorFollower);
		logging();
	}

	/**
	 * SmartDashBoard Logging
	 */
	public void logging() {
		SmartDashboard.putNumber("Upper Falcon Current", upperMotor.getStatorCurrent().getValue());
		SmartDashboard.putNumber("Upper Falcon Rotations", upperMotor.getPosition().getValue());
		SmartDashboard.putNumber("Lower Kraken Encoder", lowerMotor.getPosition().getValue());
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
		//lowerMotor.set(speed);
		manualOverrideActive = speed != 0;
	}

	/**
	 * Returns motor current from upper (higher geared) kraken
	 * @return
	 */
	public double getMotorCurrent() {
		return upperMotor.getStatorCurrent().getValue();
	}

	/**
	 * Returns whether the higher geared climb motor current is above the threshold for the limit switch
	 * @return At Current Limit
	 */
	public boolean atCurrentLimit() {
		return getMotorCurrent() > Constants.Climb.currentLimit;
	}
	public boolean useClosedLoop(){
		return !atCurrentLimit() && !manualOverrideActive;
	}

	/**
	 * Gets the current position of the higher geared Talon on the Climb in Rotations
	 * May be Inverted
	 * @return climb motor position in rotations
	 */
	public double getPosition(){
		return upperMotor.getPosition().getValue();
	}

	/**
	 * Sets motors to the climb to coast
	 */
	public void coast(){
		upperMotor.setControl(new CoastOut());
	}

	/**
	 * Do not use this irresponsibly
	 */
	public void resetClimbZero() {
		upperMotor.setPosition(0);
	}
}
