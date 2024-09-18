package org.sciborgs1155.robot.shooter;
import static org.sciborgs1155.robot.Ports.Wheels.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RealWheel implements WheelIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public RealWheel(int id, boolean inverted) {
        motor = new CANSparkMax(id, MotorType.kBrushless);
        motor.setInverted(inverted);
        encoder = motor.getEncoder();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public double getSpeed() {
        return encoder.getVelocity();
    }

}
