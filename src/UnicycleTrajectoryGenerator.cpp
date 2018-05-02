/**
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 *          Giulio Romualdi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/Core/EigenHelpers.h>
#include "UnicycleTrajectoryGenerator.h"

bool UnicycleTrajectoryGenerator::clearAndAddMeasuredStep(std::shared_ptr<FootPrint> foot, Step &previousStep, const iDynTree::Vector2 &measuredPosition, double measuredAngle)
{
    foot->clearSteps();

    Step correctedStep = previousStep;

    correctedStep.position = measuredPosition;
    iDynTree::Rotation initialRotation = iDynTree::Rotation::RotZ(previousStep.angle);
    iDynTree::Rotation measuredRotation = iDynTree::Rotation::RotZ(measuredAngle);
    correctedStep.angle = previousStep.angle + (initialRotation.inverse()*measuredRotation).asRPY()(2);

    return foot->addStep(correctedStep);
}

UnicycleTrajectoryGenerator::UnicycleTrajectoryGenerator()
    :m_left(std::make_shared<FootPrint>())
    ,m_right(std::make_shared<FootPrint>())
{
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT,
                                                         const InitialState& weightInLeftAtMergePoint)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return computeNewSteps(leftFoot, rightFoot, initTime) && interpolate(*leftFoot, *rightFoot, initTime, dT, weightInLeftAtMergePoint);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return computeNewSteps(leftFoot, rightFoot, initTime) && interpolate(*leftFoot, *rightFoot, initTime, dT);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(double initTime, double dT, double endTime)
{
    m_left->clearSteps();
    m_right->clearSteps();
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolate(*m_left, *m_right, initTime, dT);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, double endTime)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolate(*m_left, *m_right, initTime, dT);
}


bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint)
{
    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint);
}

bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint,
                                             const Step &measuredLeft, const Step &measuredRight)
{
    Step previousL, previousR;

    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    m_left->getLastStep(previousL);
    m_right->getLastStep(previousR);

    m_left->clearSteps();
    m_right->clearSteps();

    if (!m_left->addStep(measuredLeft)){
        std::cerr << "The measuredLeft step is invalid." << std::endl;
        return false;
    }

    if (!m_right->addStep(measuredRight)){
        std::cerr << "The measuredRight step is invalid." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint, previousL, previousR);
}

bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint,
                                             bool correctLeft, const iDynTree::Vector2 &measuredPosition, double measuredAngle)
{
    Step previousL, previousR, correctedStep;

    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    m_left->getLastStep(previousL);
    m_right->getLastStep(previousR);

    std::shared_ptr<FootPrint> toBeCorrected = correctLeft ? m_left : m_right;

    correctedStep = correctLeft ? previousL : previousR;

    if (!clearAndAddMeasuredStep(toBeCorrected, correctedStep, measuredPosition, measuredAngle)){
        std::cerr << "Failed to update the steps using the measured value." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
            interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint, previousL, previousR);
}

bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint, const iDynTree::Vector2 &measuredLeftPosition, double measuredLeftAngle, const iDynTree::Vector2 &measuredRightPosition, double measuredRightAngle)
{
    Step previousL, previousR;

    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    m_left->getLastStep(previousL);
    m_right->getLastStep(previousR);

    if (!clearAndAddMeasuredStep(m_left, previousL, measuredLeftPosition, measuredLeftAngle)){
        std::cerr << "Failed to update the left steps using the measured value." << std::endl;
        return false;
    }

    if (!clearAndAddMeasuredStep(m_right, previousR, measuredRightPosition, measuredRightAngle)){
        std::cerr << "Failed to update the right steps using the measured value." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint, previousL, previousR);
}


// DCM functions

bool UnicycleTrajectoryGenerator::generateAndInterpolateDCM(double initTime, double dT, double endTime)
{
    m_left->clearSteps();
    m_right->clearSteps();
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolateDCM(*m_left, *m_right, initTime, dT);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolateDCM(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, double endTime)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolateDCM(*m_left, *m_right, initTime, dT);
}


bool UnicycleTrajectoryGenerator::reGenerateDCM(double initTime, double dT, double endTime, const DCMInitialState &DCMBoundaryConditionAtMergePoint)
{
    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolateDCM(*m_left, *m_right, initTime, dT, DCMBoundaryConditionAtMergePoint);
}

bool UnicycleTrajectoryGenerator::reGenerateDCM(double initTime, double dT, double endTime,
                                                const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointPosition,
                                                const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointVelocity,
                                                bool correctLeft,
                                                const iDynTree::Vector2 &measuredPosition, double measuredAngle)
{
    // set the boundary conditions
    DCMInitialState DCMBoundaryConditionAtMergePoint;
    DCMBoundaryConditionAtMergePoint.initialPosition = DCMBoundaryConditionAtMergePointPosition;
    DCMBoundaryConditionAtMergePoint.initialVelocity = DCMBoundaryConditionAtMergePointVelocity;

    // get the right and the foot last steps before the initTime
    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    Step correctedStep;
    correctedStep.position = measuredPosition;

    if(correctLeft){
        Step previousL;
        m_left->getLastStep(previousL);
        // clear all the steps of the corrected foot
        m_left->clearSteps();

        // get the impact time
        correctedStep.impactTime = previousL.impactTime;

        // wrap the angle
        iDynTree::Rotation initialRotation = iDynTree::Rotation::RotZ(previousL.angle);
        iDynTree::Rotation measuredRotation = iDynTree::Rotation::RotZ(measuredAngle);
        correctedStep.angle = previousL.angle + (initialRotation.inverse() * measuredRotation).asRPY()(2);

        // add the steps
        if (!m_left->addStep(correctedStep)){
            std::cerr << "The measuredLeft step is invalid." << std::endl;
            return false;
        }
    }
    else{
        Step previousR;
        m_right->getLastStep(previousR);

        // clear all the steps of the corrected foot
        m_right->clearSteps();

        // get the impact time
        correctedStep.impactTime = previousR.impactTime;

        // wrap the angle
        iDynTree::Rotation initialRotation = iDynTree::Rotation::RotZ(previousR.angle);
        iDynTree::Rotation measuredRotation = iDynTree::Rotation::RotZ(measuredAngle);
        correctedStep.angle = previousR.angle +  (initialRotation.inverse() * measuredRotation).asRPY()(2);

        // add the steps
        if (!m_right->addStep(correctedStep)){
            std::cerr << "The measuredLeft step is invalid." << std::endl;
            return false;
        }
    }

    bool ok = true;
    ok = ok && setEndTime(endTime);
    ok = ok && computeNewSteps(m_left, m_right, initTime);

    // if the following it is true the robot should stop.
    // if(m_left->numberOfSteps() + m_right->numberOfSteps() == 3)
    // {
    //     // find the swing foot
    //     auto swingFoot = m_left->numberOfSteps() == 2 ? m_left : m_right;
    //     auto stanceFoot = m_left->numberOfSteps() != 2 ? m_left : m_right;
    //     StepList stepList = swingFoot->getSteps();

    //     // get the last footprint of the swing foot
    //     Step lastStep;
    //     if(!swingFoot->getLastStep(lastStep)){
    //         std::cerr << "Unable to get the last step." << std::endl;
    //         return false;
    //     }

    //     iDynTree::Vector2 displacement;
    //     iDynTree::toEigen(displacement) = iDynTree::toEigen(stepList.back().position) - iDynTree::toEigen(stepList.front().position);

    //     // the last footstep is removed
    //     if(!swingFoot->removeLastStep()){
    //         std::cerr << "Unable to remove the last step." << std::endl;
    //         return false;
    //     }

    //     // the new footprint will have the same impact time and angle of the previous one while it is shifted forward
    //     double duration = stepList.back().impactTime  - stepList.front().impactTime;
    //     iDynTree::toEigen(lastStep.position) = iDynTree::toEigen(lastStep.position) + iDynTree::toEigen(displacement);
    //     // lastStep.impactTime = stanceFoot->getSteps().front().impactTime  + duration;
    //     if(!swingFoot->addStep(lastStep)){
    //         std::cerr << "Unable to add a new step." << std::endl;
    //         return false;
    //     }

    //     double impactTime = lastStep.impactTime;
    //     if(!stanceFoot->getLastStep(lastStep)){
    //         std::cerr << "Unable to get the last step." << std::endl;
    //         return false;
    //     }
    //     iDynTree::toEigen(lastStep.position) = iDynTree::toEigen(lastStep.position) + iDynTree::toEigen(displacement);
    //     lastStep.impactTime = impactTime  + duration;
    //     if(!stanceFoot->addStep(lastStep)){
    //         std::cerr << "Unable to add a new step." << std::endl;
    //         return false;
    //     }

    // }

    ok = ok && interpolateDCM(*m_left, *m_right, initTime, dT, DCMBoundaryConditionAtMergePoint);

    // evaluate the trajectory
    return ok;

}

bool UnicycleTrajectoryGenerator::reGenerateDCM(double initTime, double dT, double endTime,
                                                const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointPosition,
                                                const iDynTree::Vector2 &DCMBoundaryConditionAtMergePointVelocity,
                                                const iDynTree::Vector2 &measuredLeftPosition, double measuredLeftAngle,
                                                const iDynTree::Vector2 &measuredRightPosition, double measuredRightAngle)
{
    // set the boundary conditions
    DCMInitialState DCMBoundaryConditionAtMergePoint;
    DCMBoundaryConditionAtMergePoint.initialPosition = DCMBoundaryConditionAtMergePointPosition;
    DCMBoundaryConditionAtMergePoint.initialVelocity = DCMBoundaryConditionAtMergePointVelocity;

    // get the right and the foot last steps before the initTime
    Step previousL, previousR;
    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }
    m_left->getLastStep(previousL);
    m_right->getLastStep(previousR);

    // clear all the trajectory
    m_left->clearSteps();
    m_right->clearSteps();

    // the new initial steps are the measured steps
    Step measuredLeft, measuredRight;
    measuredLeft.impactTime = previousL.impactTime;
    measuredLeft.position = measuredLeftPosition;
    measuredLeft.angle = measuredLeftAngle;

    measuredRight.impactTime = previousR.impactTime;
    measuredRight.position = measuredRightPosition;
    measuredRight.angle = measuredRightAngle;

    // add the steps
    if (!m_left->addStep(measuredLeft)){
        std::cerr << "The measuredLeft step is invalid." << std::endl;
        return false;
    }

    if (!m_right->addStep(measuredRight)){
        std::cerr << "The measuredRight step is invalid." << std::endl;
        return false;
    }

    // evaluate the trajectory
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolateDCM(*m_left, *m_right, initTime, dT, DCMBoundaryConditionAtMergePoint);
}
