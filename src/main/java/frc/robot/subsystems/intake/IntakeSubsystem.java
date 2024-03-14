package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.LaserCANSensor;

public class IntakeSubsystem extends SubsystemBase {
	final private CANSparkMax intakeMotorUpper;
	final private CANSparkMax intakeMotorLower;

	public IntakeSubsystem() {
		intakeMotorUpper = new CANSparkMax(Constants.Intake.upperNeoID, CANSparkLowLevel.MotorType.kBrushless);
		intakeMotorLower = new CANSparkMax(Constants.Intake.lowerNeoID, CANSparkLowLevel.MotorType.kBrushless);
		intakeMotorUpper.setIdleMode(CANSparkBase.IdleMode.kCoast);
		intakeMotorLower.setIdleMode(CANSparkBase.IdleMode.kCoast);
		intakeMotorLower.setInverted(true);
	}



	public void run(double speedLower, double speedUpper){
//		if (speed == 0) {
//			intakeMotor1.set(speed);
//			intakeMotor2.set(speed);
//		} else if (speed == 0 && laser.getWeightedDistance() > 70) {
//			intakeMotor1.set(speed);
//			intakeMotor2.set(speed);
//		}

		intakeMotorUpper.set(speedUpper);
		intakeMotorLower.set(speedLower);
	}
}
