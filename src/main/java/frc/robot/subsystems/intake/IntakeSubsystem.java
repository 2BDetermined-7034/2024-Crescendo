package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;

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
		Constants.Shooter.isLaserIntakeTriggered = laserIntake.getWeightedDistance() >= 70.0;
		Constants.Shooter.isLaserShooterTriggered = laserShooter.getWeightedDistance() >= 70.0;
		if (!(Constants.Shooter.isLaserIntakeTriggered && Constants.Shooter.isLaserShooterTriggered)) {
			if (Constants.Shooter.isShooterAtHome) {
				intakeMotorUpper.set(speedUpper);
			}
			intakeMotorLower.set(speedLower);
		}
	}
}
