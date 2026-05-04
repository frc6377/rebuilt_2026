    @Override
    public void updateInputs(BaseShooterIOInputs inputs) {}

    @Override
    public void setFlywheelVelocity(AngularVelocity velocity) {}

    @Override
    public void setFlywheelVoltage(Voltage voltage) {}

    @Override
    public void stop() {
        flywheel.stop();
    }
}
