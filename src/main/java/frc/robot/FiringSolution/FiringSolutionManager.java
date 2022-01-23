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
        for (int i = 0; i < solutions.size(); i++) {
            if (solutions.get(i).getDistance() == distance) {
                break;
            }
            if (solutions.get(i).getDistance() > distance) { // iterate through ArrayList until desired position found
                solutions.add(i, add);
                break;
            }
        }
    }

    public FiringSolution calcNewSolution(int currentDistance) {
        int upper = -1; // position of upper value (initialized to -1 as -1 is not a valid list index)
        int lower = -1; // position of lower value

        for (int i = 0; i < solutions.size(); i++) {
            if (solutions.get(i).getDistance() == currentDistance) {
                return solutions.get(i);
            }
            if (solutions.get(i).getDistance() > currentDistance) {
                upper = i;
                lower = i-1;
                break;
            }
        }

        if ((upper == -1) || (lower == -1)) {
            return null; // not sure what the proper thing to return here is
        } else {
            double d = (currentDistance - solutions.get(lower).getDistance()) / (solutions.get(upper).getDistance() - solutions.get(lower).getDistance());
            return new FiringSolution(
                (int) (solutions.get(upper).gethoodPosition() * d + solutions.get(lower).gethoodPosition() * (1 - d)),
                (int) (solutions.get(upper).getflywheelSpeed() * d + solutions.get(lower).getflywheelSpeed() * (1 - d)) // new firing solution w/o distance
            );
        }

    }

    public static FiringSolutionManager getSingleton() {
        return singleton;
    }
}