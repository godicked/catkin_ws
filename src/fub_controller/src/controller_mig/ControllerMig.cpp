#include "ControllerMig.h"

#include <ros/ros.h>

#include <limits>
#include <string.h>

// in controlSteering only
#include <boost/algorithm/clamp.hpp>
#include <tf/tf.h>
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

namespace fub {
namespace controller {
namespace mig {

using namespace fub::controller::util;

void ControllerMig::onInit()
{
    mSteeringAngleNormalized=0.0;
    // advertise publishers (before subscribing to the vehicle state / setting up timer)
    mWantedNormSteerAnglePublisher = getNodeHandle().advertise<std_msgs::Float32>("command/normalized_steering_angle", 10);
    mWantedSpeedPublisher          = getNodeHandle().advertise<std_msgs::Int16>("manual_control/speed", 10);
    mWantedSteeringAnglePublisher  = getNodeHandle().advertise<std_msgs::Int16>("manual_control/steering", 10);


    // set up dynamic reconfigure service (before any configuration is used) - this will start the timer
    setupConfiguration("Main", getPrivateNodeHandle());



    // initialize the sub-components (before subscribers)
    mPathFollower.setup(getPrivateNodeHandle().param<std::string>("odom_frame", "odom"), getNodeHandle(), getPrivateNodeHandle());
    mSteerPIDController.setup(getNodeHandle(), getPrivateNodeHandle());

    // register the subscribers that receive the vehicle state information
    mVehicleState.setup(getNodeHandle());
}


// default parameter set
void ControllerMig::configurationCallback(fub_controller::ControllerMigConfig & config, uint32_t level) {
    DynamicConfigurationHelper::configurationCallback(config, level);

    // create / update up the timer
    ros::Duration timerPeriod = ros::Duration(1.0 / mConfig.execution_frequency);
    if (not mTimer.isValid()) {
        mTimer = getNodeHandle().createTimer(timerPeriod, &ControllerMig::callbackTimer, this);
    } else {
        mTimer.setPeriod(timerPeriod);
    }
}


void ControllerMig::callbackTimer(const ros::TimerEvent &)
{
    // create a copy of the vehicle state - we do NOT want these values to
    // change while we are working with them
    // TODO: ensure that data does not change during copying
    VehicleState currentVehicleState = mVehicleState;

    update(currentVehicleState);
}


void ControllerMig::update(VehicleState const & currentVehicleState)
{
    // if we have no path, don't do anything
    if (currentVehicleState.mPath.trajectory.empty()) {
        ROS_DEBUG_THROTTLE(1, "Controller has no path");

        //publish last/default messages
        //TODO unset activation? brake?
        publish(currentVehicleState);

        return;
    }

    // TODO: should we assume a fixed frequency, or rather track the delta
    //       time ourselves?
    double deltaT = 1.0 / (mConfig.execution_frequency);

    //update spline for path following
    mPathFollower.updateSplineAndClosestPoints(currentVehicleState);

    // get bearing angle
    double const angleBearing = mPathFollower.getBearingAngle(currentVehicleState);

    //get wanted velocity
    double const wantedSpeed = mPathFollower.getWantedVelocity(currentVehicleState);


    // steer correction
    double angleHead = currentVehicleState.getVehicleYaw();
    if (not std::isnan(angleHead) and not std::isnan(angleBearing)) { // TODO proper safety checks
        mSteeringAngleNormalized = mSteerPIDController.control(currentVehicleState, deltaT, angleBearing, angleHead);
    } else {
        // TODO: should we disengage?
        ROS_ERROR("angleHead or angleBearing is nan, driving straight");
        ROS_ERROR("angleBearing: %f angleHead: %f current speed: %f", angleBearing, angleHead, currentVehicleState.mCurrentSpeedFrontAxleCenter);
        mSteeringAngleNormalized = 0;
    }
        //ROS_INFO("steerOutput%f",mSteeringAngleNormalized);

    // TODO: this makes no sense: shouldn't this be mGear or better yet, the
    //       current gear?
/*
    if (mDriveGear == fub_mig_can_msgs::MIGGearStatus::GEAR_POSITION_R) { //Drive Backwards
        // when driving backwards, we need to increase our steering angle to keep
        // the rear axle center point following the plan - the factor of 3.0 was
        // determined experimentally
        steerAngle = steerAngle * 3.0;
    }
*/
    //ROS_INFO("x,y = %.3f, %.3f v_cur %.3f, v_req %.3f steer %.3f ",currentVehicleState.mVehiclePosition[0],currentVehicleState.mVehiclePosition[1],\
    currentVehicleState.mCurrentSpeedFrontAxleCenter, wantedSpeed, mSteeringAngleNormalized);
    publish(currentVehicleState);

    publishWantedSpeedAndFrontWheelAngle(wantedSpeed, mSteeringAngleNormalized);
    //TODO Remove after debug
    int16_t speed_pub =  static_cast<int16_t>(wantedSpeed *(-308.26));
    int16_t steer_pub =  static_cast<int16_t>(80.0 * mSteeringAngleNormalized) + 81;
    // ROS_INFO("Commanded Values: steer_pb %d  norm_str %.3f   speed_pb %d   raw_spd %.3f ",steer_pub,mSteeringAngleNormalized,speed_pub, wantedSpeed);
    // ROS_INFO("cur_state:  x,y %.3f,%.3f   vel: %.3f  yaw:%.3f odom_time %.3f",currentVehicleState.mVehiclePosition[0],\
              currentVehicleState.mVehiclePosition[1], currentVehicleState.mCurrentSpeedFrontAxleCenter,currentVehicleState.getVehicleYaw(),\
               currentVehicleState.mLastOdomTimeStampReceived.toSec());
    //TODO remove till here
}

void ControllerMig::publish(VehicleState const & vehicleState)
{
    ros::Time now = ros::Time::now();

    std_msgs::Float32 steerMsg;
    steerMsg.data = mSteeringAngleNormalized;
    mWantedNormSteerAnglePublisher.publish(steerMsg);
}

void ControllerMig::publishWantedSpeedAndFrontWheelAngle(double speed, double wheelAngle)
{
    // publish wanted speed
        std_msgs::Int16 wantedSpeedMsg;
        //TODO korivi . updating max speed to 308 from 1489.36
        // This conversion is made on basis that speed is calculated in mps and the output to the motor is in rpm of wheel.
        /* TODO - verify these conversions - this is basis for chosing 308
        1000rpm for motor - then
        wheel speed max =  1000/(5.5*60) = 3.03rot per sec => 3.03*2*3.14*0.031 = 0.5899 mps
        if 1000rpm is wheel speed then
        wheel speed max  =  1000/60 ==> 3.244 mps. From simulation - this is correct
        if 3.24 mps is max velocity, whats the time I want to attain it?
        lets say in 5s then -> 3.24/5 = 0.65 mpss
        min max accc =  [-0.65, 0.65] lets limit it to [-0.5:0.5]
        1m - 5.1366 rotations of wheel.
        */
        wantedSpeedMsg.data        = static_cast<int16_t>(speed *(-308.26));
        mWantedSpeedPublisher.publish(wantedSpeedMsg);

        // publish wanted steering angle
        std_msgs::Int16 wantedAngleMsg;
        //TODO : Korivi - the eqn (90.0 * wheelAngle) + 90 changed to below for making them suitable for the model car
        //TODO - looks like the original 0 is left and 180 is right, simulator is reservse. change this
        wantedAngleMsg.data = static_cast<int16_t>(90- 90.0 * wheelAngle);
        mWantedSteeringAnglePublisher.publish(wantedAngleMsg);
}

}
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fub::controller::mig::ControllerMig, nodelet::Nodelet);
