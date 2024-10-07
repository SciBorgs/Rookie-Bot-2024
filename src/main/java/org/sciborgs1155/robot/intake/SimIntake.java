package org.sciborgs1155.robot.intake;

public class SimIntake implements IntakeIO{

    public SimIntake() {
        
    }

    @Override
    public void setRoller(double speed) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getPositon() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getVelocity() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void setWrist(double volts) {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    
}
