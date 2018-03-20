/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLETRAJECTORYGENERATOR_H
#define UNICYCLETRAJECTORYGENERATOR_H

#include "UnicyclePlanner.h"
#include "FootPrintsInterpolator.h"
#include <memory>

class UnicycleTrajectoryGenerator : public UnicyclePlanner, public FeetInterpolator
{
    std::shared_ptr<FootPrint> m_left, m_right;

    bool clearAndAddMeasuredStep(std::shared_ptr<FootPrint> foot, Step& previousStep, const iDynTree::Vector2 &measuredPosition, double measuredAngle);

public:
    UnicycleTrajectoryGenerator();

    //DO NOT FORGET TO CALL ALL THE INITIALIZATION METHODS OF BOTH FEETINTERPOLATOR AND UNICYCLEPLANNER

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT,
                                const InitialState &weightInLeftAtMergePoint); //both feet are supposed to start on the ground at zero velocity. The initTime must be greater than the maximum of the first impactTime of the two feet. The first step has half switch time. The FootPrints needs to be ordered!

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT);

    bool generateAndInterpolate(double initTime, double dT, double endTime);

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, double endTime);

    bool reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint);

    bool reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint,
                    const Step &measuredLeft, const Step &measuredRight);

    bool reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint, bool correctLeft,
                    const iDynTree::Vector2 &measuredPosition, double measuredAngle);
    bool reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint,
                    const iDynTree::Vector2 &measuredLeftPosition, double measuredLeftAngle,
                    const iDynTree::Vector2 &measuredRightPosition, double measuredRightAngle);


    /**
     * Generate a new trajectory. (With this method the ZMP trahectory is not evaluated).
     * @param initTime initial time of the trajectory;
     * @param dT sampling time;
     * @param endTime final time of the trajectory.
     * @return true/false in case of success/failure
     */
    bool generateAndInterpolateDCM(double initTime, double dT, double endTime);

    /**
     * Generate a new trajectory. (With this method the ZMP trahectory is not evaluated).
     * @param leftFoot pointer of the left footsteps (it will be filled by this method);
     * @param rightFoot pointer of the right footsteps (it will be filled by this method);
     * @param initTime initial time of the trajectory;
     * @param dT sampling time;
     * @param endTime final time of the trajectory.
     * @return true/false in case of success/failure
     */
    bool generateAndInterpolateDCM(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot,
                                   double initTime, double dT, double endTime);

    /**
     * Re-generate a new trajectory. (With this method the ZMP trahectory is not evaluated).
     * This method can be called only if the `generateAndInterpolateDCM()` was already called.
     * @param initTime initial time of the trajectory;
     * @param dT sampling time;
     * @param endTime final time of the trajectory;
     * @param DCMBoundaryConditionAtMergePoint desired position and velocity at the beginning of the trajectory.
     * @return true/false in case of success/failure
     */
    bool reGenerateDCM(double initTime, double dT, double endTime,
                       const DCMInitialState &DCMBoundaryConditionAtMergePoint);

    /**
     * Re-generate a new trajectory. (With this method the ZMP trahectory is not evaluated).
     * This method can be called only if the `generateAndInterpolateDCM()` was already called.
     * With this function you can evaluate a new trajectory from the real position of the left and right feet.
     * @param initTime initial time of the trajectory;
     * @param dT sampling time;
     * @param endTime final time of the trajectory;
     * @param DCMBoundaryConditionAtMergePointPosition desired position at the beginning of the trajectory;
     * @param DCMBoundaryConditionAtMergePointVelocity desired velocity at the beginning of the trajectory;
     * @param measuredLeftPosition real position of the left foot;
     * @param measuredLeftAngle real attitude of the left foot;
     * @param measuredRightPosition real position of the right foot;
     * @param measuredRightAngle real attitude of the right foot;
     * @return true/false in case of success/failure
     */
    bool reGenerateDCM(double initTime, double dT, double endTime,
                       const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointPosition,
                       const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointVelocity,
                       const iDynTree::Vector2 &measuredLeftPosition, double measuredLeftAngle,
                       const iDynTree::Vector2 &measuredRightPosition, double measuredRightAngle);

    /**
     * Re-generate a new trajectory. (With this method the ZMP trahectory is not evaluated).
     * This method can be called only if the `generateAndInterpolateDCM()` was already called.
     * With this function you can evaluate a new trajectory from the real position of the left or right feet.
     * @param initTime initial time of the trajectory;
     * @param dT sampling time;
     * @param endTime final time of the trajectory;
     * @param DCMBoundaryConditionAtMergePointPosition desired position at the beginning of the trajectory;
     * @param DCMBoundaryConditionAtMergePointVelocity desired velocity at the beginning of the trajectory;
     * @param correctLeft boolean used to understand if the left or the right foot has to be corrected
     * if it is true the left foot will be corrected (i.e. the left foot is the stance foot);
     * @param measuredPosition real position of the foot (left if correctLeft is true);
     * @param measuredAngle real attitude of the foot (left if correctLeft is true).
     * @return true/false in case of success/failure.
     */
    bool reGenerateDCM(double initTime, double dT, double endTime,
                       const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointPosition,
                       const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointVelocity,
                       bool correctLeft,
                       const iDynTree::Vector2 &measuredPosition, double measuredAngle);
};

#endif // UNICYCLETRAJECTORYGENERATOR_H
