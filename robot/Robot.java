public class Robot {
    public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
        return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
    }
    Drive drive = new Drive();

    private void configureBindings() {
        drive.setDefaultCommand(drive.drive(driver::getLeftY, driver::getRightY));
    }    
}
