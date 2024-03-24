package frc.robot.utils;

import frc.robot.subsystems.vision_sys.VisionVariables;

public class KalmanFilter {
    //Limelight distance is very jittry, so we need to tell the filter that there is a maximum that
    //values can change over the course of a set amount of time. So for example, in 20ms the distance cannot 
    //just immedietly jump 1 foot away, so we set a variance on the filter to say "hey any values past this variance are basically errors"
    //That is in essence what the filter does
    //We do not need to apply this filter for pigeon because that thing is basically perfect
    //Once we filter the limelight, use those distances with the pigeon to fuse for a near perfect localization of the robot (taking weighted averages)

    double limelightVarianceX = 0; //We must find through a bunch of trial and error of testing
    double previousReadingX; //We need to compare the previous reading of the limelight to the current one
    double gainGuessX; //this value basically tells us based on the previous and current reading how much in theory the next distance should increase or decrease
    double previousLimelightVarianceX = 0; //get the old limelight variance
    double totalCovarianceX = 0; //using the previous and new variance get the average or total variance
    double actualReadingX = 0; //what we are going to return after filtering

    double limelightVarianceY = 0;
    double previousReadingY;
    double gainGuessY;
    double previousLimelightVarianceY = 0;
    double totalCovarianceY = 0;
    double actualReadingY = 0;

    public double filterNoiseLimelightX(double limelightX, double pigeonX) { 
        limelightVarianceX = previousLimelightVarianceX + totalCovarianceX;
        actualReadingX = previousReadingX + limelightX;
        gainGuessX = totalCovarianceX/(totalCovarianceX + limelightVarianceX);
        actualReadingX = actualReadingX + gainGuessX * (limelightX - actualReadingX);
        limelightX = (1 - gainGuessX) * limelightVarianceX;
        previousReadingX = actualReadingX;
        previousLimelightVarianceX = limelightVarianceX;
        
        return actualReadingX;
    }

    public double filterNoiseLimelightY(double limelightY, double pigeonY) {
        
        limelightVarianceY = previousLimelightVarianceY + totalCovarianceY;
        actualReadingY = previousReadingY + limelightY;
        gainGuessY = totalCovarianceY/(totalCovarianceY + limelightVarianceY);
        actualReadingY = actualReadingY + gainGuessY * (limelightY - actualReadingY);
        limelightY = (1 - gainGuessY) * limelightVarianceY;
        previousReadingY = actualReadingY;
        previousLimelightVarianceY = limelightVarianceY;
        
        return actualReadingY;
    }

    public double fusePoitionX(double pigeonX) {
        //Weighted average of the pigeon and limelight: 30% limelight, 70% pigeon

        double limelightX = filterNoiseLimelightX(VisionVariables.BackCam.target.getX(), pigeonX);

        double limelightWeightX = limelightX * .3;
        double pigeonWeightX = pigeonX * .7;

        double weightedAverageX = limelightWeightX + pigeonWeightX;

        return weightedAverageX;
    }

    public double fusePoitionY(double pigeonY) {
        //Weighted average of the pigeon and limelight: 30% limelight, 70% pigeon

        double limelightY = filterNoiseLimelightY(VisionVariables.BackCam.target.getY(), pigeonY);

        double limelightWeightY = limelightY * .3;
        double pigeonWeightY = pigeonY * .7;

        double weightedAverageY = limelightWeightY + pigeonWeightY;

        return weightedAverageY;
    }
}