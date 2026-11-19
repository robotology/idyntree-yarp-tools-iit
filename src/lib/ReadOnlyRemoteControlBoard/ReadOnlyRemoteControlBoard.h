/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
* Author: Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H
#define YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H

#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IAmplifierControl.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/os/Semaphore.h>

#include "stateExtendedReader.h"

#include <vector>
#include <iostream>
namespace yarp {
        namespace dev {
            class ReadOnlyRemoteControlBoard;

        }
}

/*!
 * @brief Readonly version of the remote control board.
 *
 * It opens only interfaces though for providing data, not for commanding the robot.
 * Information usually obtained though calls to the underlining device are expected to be
 * provided by the configuration parameters
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | remote     |      -         | string  | -      |   -           | Yes     | Prefix of the remote port which this device will connect. The port opened will be "{remote}/stateExt:o" |  |
 * | local     |      -         | string  | -      |   -           | Yes     | Prefix of the ports opened by this device |  |
 * | carrier     |      -         | string  | -      |   udp           | No     | Protocol to be used during the connection. Default to udp |  |
 * | axesDescription     |      -         | vector of pairs of string and vocab  | -      |   -           | Yes     | Description of the axes managed by this control board. Each element is a pair: a string denoting the axis name and a vocab denoting the axis type (rotational, prismatic, etc). |  |
 *
 */
class yarp::dev::ReadOnlyRemoteControlBoard
: public yarp::dev::IEncodersTimed
, public yarp::dev::DeviceDriver
, public yarp::dev::IAxisInfo
, public yarp::dev::IMotorEncoders
, public yarp::dev::ITorqueControl
, public yarp::dev::IPWMControl
, public yarp::dev::IAmplifierControl
, public yarp::dev::IPositionControl
, public yarp::dev::IVelocityControl
, public yarp::dev::IControlMode
{

#ifndef DOXYGEN_SHOULD_SKIP_THIS

    // Buffer associated to the extendedOutputStatePort port; in this case we will use the type generated
    // from the YARP .thrift file
    StateExtendedInputPort m_extendedIntputStatePort;  // Buffered port storing new data
    yarp::os::Semaphore m_extendedPortMutex;

    std::vector<std::pair<std::string, yarp::dev::JointTypeEnum> > m_axes;

    mutable Stamp m_lastStamp;  //this is shared among all calls that read encoders
    // Semaphore mutex;
    int m_numberOfJoints;

#endif /*DOXYGEN_SHOULD_SKIP_THIS*/

public:
    /**
     * Constructor.
     */
    ReadOnlyRemoteControlBoard();

    /**
     * Destructor.
     */
    ~ReadOnlyRemoteControlBoard();

    virtual bool open();

    virtual bool open(yarp::os::Searchable& config);

    virtual bool close();

    bool getSingleData(int field, int j, double* val);
    bool getVectorData(int field, double* val);
    bool getTimedSingleData(int field, int j, double* val, double* time);
    bool getTimedVectorData(int field, double* val, double* time);

    /* IEncodersTimed */
    virtual yarp::dev::ReturnValue getAxes(size_t& ax);
    virtual yarp::dev::ReturnValue resetEncoder(int j) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue resetEncoders() {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setEncoder(int j, double val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setEncoders(const double *vals) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getEncoder(int j, double *v);
    virtual yarp::dev::ReturnValue getEncoderTimed(int j, double *v, double *t);
    virtual yarp::dev::ReturnValue getEncoders(double *encs);
    virtual yarp::dev::ReturnValue getEncodersTimed(double *encs, double *ts);
    virtual yarp::dev::ReturnValue getEncoderSpeed(int j, double *sp);
    virtual yarp::dev::ReturnValue getEncoderSpeeds(double *spds);
    virtual yarp::dev::ReturnValue getEncoderAcceleration(int j, double *acc);
    virtual yarp::dev::ReturnValue getEncoderAccelerations(double *accs);

    /* IAxisInfo */
    virtual yarp::dev::ReturnValue getAxisName(int j, std::string &name);
    virtual yarp::dev::ReturnValue getJointType(int j, yarp::dev::JointTypeEnum &type);

    /* IMotorEncoders */
    virtual yarp::dev::ReturnValue getNumberOfMotorEncoders(int *num);
    virtual yarp::dev::ReturnValue resetMotorEncoder(int m) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue resetMotorEncoders() {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setMotorEncoderCountsPerRevolution(int m, const double cpr) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getMotorEncoderCountsPerRevolution(int m, double *cpr) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setMotorEncoder(int m, const double val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setMotorEncoders(const double *vals) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getMotorEncoder(int m, double *v);
    virtual yarp::dev::ReturnValue getMotorEncoders(double *encs);
    virtual yarp::dev::ReturnValue getMotorEncodersTimed(double *encs, double *time);
    virtual yarp::dev::ReturnValue getMotorEncoderTimed(int m, double *encs, double *time);
    virtual yarp::dev::ReturnValue getMotorEncoderSpeed(int m, double *sp);
    virtual yarp::dev::ReturnValue getMotorEncoderSpeeds(double *spds);
    virtual yarp::dev::ReturnValue getMotorEncoderAcceleration(int m, double *acc);
    virtual yarp::dev::ReturnValue getMotorEncoderAccelerations(double *accs);

    /* ITorqueControl */
    virtual yarp::dev::ReturnValue getRefTorques(double *t) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getRefTorque(int j, double *t) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setRefTorques(const double *t) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setRefTorque(int j, double t) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setRefTorques(const int n_joint, const int *joints, const double *t) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setMotorTorqueParams(int j,  const yarp::dev::MotorTorqueParameters params) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getTorque(int j, double *t);
    virtual yarp::dev::ReturnValue getTorques(double *t);
    virtual yarp::dev::ReturnValue getTorqueRange(int j, double *min, double *max) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getTorqueRanges(double *min, double *max) {return ReturnValue_error_not_implemented_by_device;}


    /* IPWMControl */
    virtual yarp::dev::ReturnValue getNumberOfMotors(int *number);
    virtual yarp::dev::ReturnValue setRefDutyCycle(int m, double ref) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setRefDutyCycles(const double *refs) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getRefDutyCycle(int m, double *ref) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getRefDutyCycles(double *refs) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getDutyCycle(int m, double *val);
    virtual yarp::dev::ReturnValue getDutyCycles(double *vals);

    /* IAmplifierControl */
    virtual yarp::dev::ReturnValue enableAmp(int j) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue disableAmp(int j) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getAmpStatus(int *st) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getAmpStatus(int j, int *v) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getCurrents(double *vals);
    virtual yarp::dev::ReturnValue getCurrent(int j, double *val);
    virtual yarp::dev::ReturnValue getMaxCurrent(int j, double *v) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setMaxCurrent(int j, double v) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getNominalCurrent(int m, double *val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setNominalCurrent(int m, const double val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getPeakCurrent(int m, double *val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setPeakCurrent(int m, const double val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getPWM(int j, double* val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getPWMLimit(int j, double* val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue setPWMLimit(int j, const double val) {return ReturnValue_error_not_implemented_by_device;}
    virtual yarp::dev::ReturnValue getPowerSupplyVoltage(int j, double* val) {return ReturnValue_error_not_implemented_by_device;}

    /* IPositionControl */
    yarp::dev::ReturnValue positionMove(int j, double ref) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue positionMove(const int n_joint, const int *joints, const double *refs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue positionMove(const double *refs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTargetPosition(const int joint, double *ref) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTargetPositions(double *refs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTargetPositions(const int n_joint, const int *joints, double *refs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue relativeMove(int j, double delta) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue relativeMove(const int n_joint, const int *joints, const double *refs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue relativeMove(const double *deltas) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue checkMotionDone(int j, bool& flag) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue checkMotionDone(const std::vector<int>& joints, bool& flag) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue checkMotionDone(bool& flag) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setTrajSpeed(int j, double sp) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setTrajSpeeds(const int n_joint, const int *joints, const double *spds) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setTrajSpeeds(const double *spds) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setTrajAcceleration(int j, double acc) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setTrajAccelerations(const int n_joint, const int *joints, const double *accs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setTrajAccelerations(const double *accs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTrajSpeed(int j, double *ref) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTrajSpeeds(const int n_joint, const int *joints, double *spds) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTrajSpeeds(double *spds) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTrajAcceleration(int j, double *acc) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTrajAccelerations(const int n_joint, const int *joints, double *accs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTrajAccelerations(double *accs) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue stop(int j) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue stop(const int n_joint, const int *joints) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue stop() {return ReturnValue_error_not_implemented_by_device;}

    /* IVelocityControl */
    yarp::dev::ReturnValue velocityMove(int j, double v) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue velocityMove(const double *v) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue velocityMove(const int n_joint, const int *joints, const double *spds) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTargetVelocity(const int joint, double* vel) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTargetVelocities(double* vels) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getTargetVelocities(const int n_joint, const int* joints, double* vels) {return ReturnValue_error_not_implemented_by_device;}

    /* IControlMode */
    yarp::dev::ReturnValue getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum>& avail) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getControlMode(int j, yarp::dev::ControlModeEnum& mode) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getControlModes(std::vector<yarp::dev::ControlModeEnum>& mode) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue getControlModes(const std::vector<int>& joints, std::vector<yarp::dev::ControlModeEnum>& mode) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setControlMode(int j, yarp::dev::SelectableControlModeEnum mode) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setControlModes(const std::vector<int>& joints, const std::vector<yarp::dev::SelectableControlModeEnum>& mode) {return ReturnValue_error_not_implemented_by_device;}
    yarp::dev::ReturnValue setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum>& mode) {return ReturnValue_error_not_implemented_by_device;}


};

#if defined(_MSC_VER) && !defined(YARP_NO_DEPRECATED) // since YARP 2.3.65
YARP_WARNING_POP
#endif


#endif // YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H
