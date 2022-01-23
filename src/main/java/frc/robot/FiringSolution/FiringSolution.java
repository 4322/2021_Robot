package frc.robot.FiringSolution;

public class FiringSolution {
        private int flywheelSpeed;
        private int hoodPosition;
        private double distance;


        public FiringSolution(int flywheelSpeed, int hoodPosition, double distance) {
            this.flywheelSpeed = flywheelSpeed;
            this.hoodPosition = hoodPosition;
            this.distance = distance;
        }

        public FiringSolution(int flywheelSpeed, int hoodPosition) {
            this.flywheelSpeed = flywheelSpeed;
            this.hoodPosition = hoodPosition;
        }

        public int getflywheelSpeed() { return flywheelSpeed; }
        public int gethoodPosition() { return hoodPosition; }
        
        public double getDistance() {
            return distance;
        }
 
}
