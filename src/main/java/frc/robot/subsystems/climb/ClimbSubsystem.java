package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
	protected TalonFX lowerMotor, upperMotor;
	protected PositionVoltage positionVoltage;
	protected double targetPosition;
	double velocity;
	public ClimbSubsystem() {
		lowerMotor = new TalonFX(Constants.Climb.lowerKrakenID, Constants.CANBUS_NAME);
		upperMotor = new TalonFX(Constants.Climb.upperKrakenID, Constants.CANBUS_NAME);

		lowerMotor.setNeutralMode(NeutralModeValue.Brake);
		upperMotor.setNeutralMode(NeutralModeValue.Brake);

//		Slot0Configs motorConfig = new Slot0Configs();
//		motorConfig.kP = 1.0;
//		motorConfig.kI = 0.0;
//		motorConfig.kD = 0.0;
//		motorConfig.kS = 0.2;
//
//		upperMotor.getConfigurator().apply(motorConfig);

//		Follower lowerMotorFollower = new Follower(Constants.Climb.upperKrakenID, false);
//		lowerMotor.setControl(lowerMotorFollower);

		positionVoltage = new PositionVoltage(0);

//		upperMotor.setPosition(0.0);


//		TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
//		TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();
//
//		/* User can optionally change the configs or leave it alone to perform a factory default */
//		leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
//		rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//
//		upperMotor.getConfigurator().apply(leftConfiguration);
//		lowerMotor.getConfigurator().apply(leftConfiguration);
//
//		/* Set up followers to follow leaders */
	}

	@Override
	public void periodic() {
//		positionVoltage.Position = targetPosition;
//		upperMotor.setControl(positionVoltage);
		lowerMotor.setControl(new Follower(upperMotor.getDeviceID(), false));
	}

	/**
	 *
	 * @param position Target position for the climb in motor revolutions
	 */
	public void setPosition(double position) {
		targetPosition = position;
	}

	/**
	 *
	 * @param speed Sets the raw velocity of the motors
	 */
	public void setSpeed(double speed) {
		upperMotor.set(speed);
		lowerMotor.set(speed);
	}

}
