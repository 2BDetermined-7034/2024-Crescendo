package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.sensors.LaserCANSensor;

public class IntakeSubsystem extends SubsystemBase {
	final private CANSparkMax intakeMotorUpper;
	final private CANSparkMax intakeMotorLower;
	LaserCANSensor laserIntake;
	LaserCANSensor laserShooter;

	public IntakeSubsystem(LaserCANSensor laserIntake, LaserCANSensor laserShooter) {
		intakeMotorUpper = new CANSparkMax(Constants.Intake.upperNeoID, CANSparkLowLevel.MotorType.kBrushless);
		intakeMotorLower = new CANSparkMax(Constants.Intake.lowerNeoID, CANSparkLowLevel.MotorType.kBrushless);
		intakeMotorUpper.setIdleMode(CANSparkBase.IdleMode.kCoast);
		intakeMotorLower.setIdleMode(CANSparkBase.IdleMode.kCoast);
		intakeMotorUpper.setInverted(true);
		this.laserIntake = laserIntake;
		this.laserShooter = laserShooter;
	}

	public void run(double speedLower, double speedUpper) {
		RobotContainer.isLaserIntakeTriggered = laserIntake.getWeightedDistance() <= 30.0;
		RobotContainer.isLaserShooterTriggered = laserShooter.getWeightedDistance() <= 30.0;
		SmartDashboard.putNumber("LaserIntake", laserIntake.getMeasurement().distance_mm);
		SmartDashboard.putNumber("LaserShooter", laserShooter.getMeasurement().distance_mm);
		if (!(RobotContainer.isLaserIntakeTriggered || RobotContainer.isLaserShooterTriggered)) {
			if (RobotContainer.isShooterAtHome) {
				intakeMotorUpper.set(speedUpper);
			}
			intakeMotorLower.set(speedLower);
		}
	}
}
