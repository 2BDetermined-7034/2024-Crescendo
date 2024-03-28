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
	protected double targetPosition;
	private Follower lowerMotorFollower;

	public ClimbSubsystem() {
		lowerMotor = new TalonFX(Constants.Climb.lowerKrakenID, Constants.CANBUS_NAME);
		upperMotor = new TalonFX(Constants.Climb.upperFalconID, Constants.CANBUS_NAME);

		lowerMotor.setNeutralMode(NeutralModeValue.Brake); // TODO SET TO BRAKE
		upperMotor.setNeutralMode(NeutralModeValue.Brake);

		lowerMotorFollower = new Follower(Constants.Climb.upperFalconID, false);

		upperMotor.setInverted(false);
		lowerMotor.setInverted(false);
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
		SmartDashboard.putNumber("Upper Falcon Rotations", upperMotor.getPosition().getValue());
		SmartDashboard.putNumber("Upper Falcon Current", upperMotor.getStatorCurrent().getValue());
		SmartDashboard.putNumber("Lower Kraken Encoder", lowerMotor.getPosition().getValue());
		SmartDashboard.putNumber("Lower Falcon Rotations", lowerMotor.getPosition().getValue());
		SmartDashboard.putNumber("Lower Falcon Current", lowerMotor.getStatorCurrent().getValue());
	}

	public void setPercent(double percent){
		upperMotor.set(percent);
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
}
