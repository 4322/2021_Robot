package frc.robot.FiringSolution;

import java.util.*;
import frc.robot.FiringSolution.FiringSolution;

public class FiringSolutionManager {

    private static FiringSolutionManager singleton;
    private ArrayList<FiringSolution> solutions;

    private FiringSolutionManager() {
        singleton = this;
        solutions = new ArrayList<FiringSolution>(); // new ArrayList to organize based on distance
    }

    public void addSolution(FiringSolution add, int distance) { 
        for (int i = 0; i < solutions.length; i++) {
            if (solutions[i].getDistance() == distance) {
                break;
            }
            if (solutions[i].getDistance() > distance) { // iterate through ArrayList until desired position found
                solutions.add(i, add);
                break;
            }
        }
    }

    public FiringSolution calcNewSolution(int currentDistance) {
        int upper = -1; // position of upper value (initialized to -1 as -1 is not a valid list index)
        int lower = -1; // position of lower value

        for (int i = 0; i < solutions.length; i++) {
            if (solutions[i].getDistance() == currentDistance) {
                return solutions[i];
            }
            if (solutions[i].getDistance() > currentDistance) {
                upper = i;
                lower = i-1;
                break;
            }
        }

        if ((upper == -1) || (lower == -1)) {
            return null; // not sure what the proper thing to return here is
        } else {
            double d = (currentDistance - solutions[lower].getDistance()) / (solutions[upper].getDistance() - solutions[lower].getDistance());
            return new FiringSolution(
                (int) (solutions[upper].gethoodPosition() * d + solutions[lower].gethoodPosition() * (1 - d)),
                (int) (solutions[upper].getflywheelSpeed() * d + solutions[lower].getflywheelSpeed() * (1 - d)) // requires new FiringSolution constructor without distance (sorry Ryan)
            );
        }

    }

    public static FiringSolutionManager getSingleton() {
        return singleton;
    }
}
