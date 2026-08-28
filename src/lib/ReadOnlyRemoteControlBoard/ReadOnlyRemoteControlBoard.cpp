/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
* Author: Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include "ReadOnlyRemoteControlBoard.h"

#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/Vector.h>


#include <algorithm>
#include <cassert>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#ifndef DOXYGEN_SHOULD_SKIP_THIS

const double TIMEOUT=0.5;

#endif /*DOXYGEN_SHOULD_SKIP_THIS*/


#if defined(_MSC_VER) && !defined(YARP_NO_DEPRECATED) // since YARP 2.3.65
// A class implementing setXxxxxMode() causes a warning on MSVC
YARP_WARNING_PUSH
YARP_DISABLE_DEPRECATED_WARNING
#endif


namespace yarp {
namespace dev {

    ReadOnlyRemoteControlBoard::ReadOnlyRemoteControlBoard()
    : m_numberOfJoints(0) {}

    /**
    * Destructor.
    */
    ReadOnlyRemoteControlBoard::~ReadOnlyRemoteControlBoard()
    {
    }

    bool ReadOnlyRemoteControlBoard::getSingleData(int field, int j, double* val)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, field, val, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getVectorData(int field, double* val)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(field, val, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getTimedSingleData(int field, int j, double* val, double* time)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, field, val, m_lastStamp, localArrivalTime);
        *time = m_lastStamp.getTime();
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getTimedVectorData(int field, double* val, double* times)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(field, val, m_lastStamp, localArrivalTime);
        std::fill_n(times, m_numberOfJoints, m_lastStamp.getTime());
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    /**
    * Default open.
    * @return always true.
    */
    bool ReadOnlyRemoteControlBoard::open() {
        return true;
    }

    bool ReadOnlyRemoteControlBoard::open(Searchable& config) {
        std::string remote = config.find("remote").asString();
        std::string local = config.find("local").asString();

        if (local.empty()) {
            yError("Problem connecting to remote controlboard, 'local' port prefix not given");
            return false;
        }

        if (remote.empty()) {
            yError("Problem connecting to remote controlboard, 'remote' port name not given");
            return false;
        }

        std::string carrier = config.check("carrier", Value("udp"), "default carrier for streaming robot state").asString();

        bool portProblem = false;
        if (!m_extendedIntputStatePort.open(local + "/stateExt:i")) {
            portProblem = true;
        }

        if (!portProblem) {
            m_extendedIntputStatePort.useCallback();
        }


        bool connectionProblem = false;
        if (!portProblem) {
            bool ok = false;
            ok = Network::connect(remote + "/stateExt:o", m_extendedIntputStatePort.getName(), carrier);
            if (!ok) {
                connectionProblem = true;
                yError("*** Extended port %s was not found on the controlBoardWrapper I'm connecting to.", (remote + "/stateExt:o").c_str());
            }
        }

        if (connectionProblem || portProblem) {
            m_extendedIntputStatePort.close();
            return false;
        }

        Value &axesDescription = config.find("axesDescription");
        if (axesDescription.isNull() || !axesDescription.isList()) {
            yError("*** Option 'axesDescription' not found or malformed.");
            m_extendedIntputStatePort.close();
            return false;
        }

        Bottle *axesDescriptionList = axesDescription.asList();
        m_numberOfJoints = axesDescriptionList->size();
        m_extendedIntputStatePort.init(m_numberOfJoints);
        m_axes.reserve(m_numberOfJoints);

        for (int index = 0; index < m_numberOfJoints; ++index) {
            const Value& axis = axesDescriptionList->get(index);
            if (axis.isNull() || !axis.isList() || axis.asList()->size() != 2) {
                yError("*** Option 'axesDescription' malformed at index %d.", index);
                m_extendedIntputStatePort.close();
                return false;
            }
            std::string axisName = axis.asList()->get(0).asString();
            JointTypeEnum axisVocab = static_cast<JointTypeEnum>(axis.asList()->get(1).asVocab32());
            m_axes.push_back(std::pair<std::string, yarp::dev::JointTypeEnum>(axisName, axisVocab));
        }

        return true;
    }

    /**
    * Close the device driver and stop the port connections.
    * @return true/false on success/failure.
    */
    bool ReadOnlyRemoteControlBoard::close() {
        m_extendedIntputStatePort.close();
        return true;
    }

    /* IEncoder */
    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getAxes(size_t& ax)
    {
        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getJointPositionSize(ax); //It is possible that the joint size provided in the configuration do not match what it is read from stateExt
        m_extendedPortMutex.post();
        return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncoder(int j, double *v)
    {
        if (j < 0 || j >= m_numberOfJoints || !v) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_ENCODER, j, v) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncoderTimed(int j, double *v, double *t)
    {
        if (j < 0 || j >= m_numberOfJoints || !v || !t) return ReturnValue_error_method_failed;

        return getTimedSingleData(VOCAB_ENCODER, j, v, t) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }


    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncoders(double *encs) {
        if (!encs) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_ENCODERS, encs) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncodersTimed(double *encs, double *ts) {
        if (!encs || !ts) return ReturnValue_error_method_failed;

        return getTimedVectorData(VOCAB_ENCODERS, encs, ts) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncoderSpeed(int j, double *sp)
    {
        if (j < 0 || j >= m_numberOfJoints || !sp) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_ENCODER_SPEED, j, sp) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }


    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncoderSpeeds(double *spds)
    {
        if (!spds) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_ENCODER_SPEEDS, spds) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncoderAcceleration(int j, double *acc)
    {
        if (j < 0 || j >= m_numberOfJoints || !acc) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_ENCODER_ACCELERATION, j, acc) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }


    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getEncoderAccelerations(double *accs)
    {
        if (!accs) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_ENCODER_ACCELERATIONS, accs) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    /* IAxisInfo */
    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getAxisName(int j, std::string& name) {
        if (j < 0 || j >= m_numberOfJoints) return ReturnValue_error_method_failed;
        name = m_axes[j].first;
        return ReturnValue_ok;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getJointType(int j, yarp::dev::JointTypeEnum& type) {
        if (j < 0 || j >= m_numberOfJoints) return ReturnValue_error_method_failed;
        type = m_axes[j].second;
        return ReturnValue_ok;
    }

    /* IMotorEncoders */
    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getNumberOfMotorEncoders(int *num)
    {
        if (!num) return ReturnValue_error_method_failed;
        *num = m_numberOfJoints;
        return ReturnValue_ok;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncoder(int m, double *v)
    {
        if (m < 0 || m >= m_numberOfJoints || !v) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_MOTOR_ENCODER, m, v) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncoderTimed(int m, double *encs, double *time)
    {
        if (m < 0 || m >= m_numberOfJoints || !encs || !time) return ReturnValue_error_method_failed;

        return getTimedSingleData(VOCAB_MOTOR_ENCODER, m, encs, time) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }


    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncoders(double *encs) {
        if (!encs) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_MOTOR_ENCODERS, encs) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncodersTimed(double *encs, double *time) {
        if (!encs || !time) return ReturnValue_error_method_failed;

        return getTimedVectorData(VOCAB_MOTOR_ENCODERS, encs, time) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncoderSpeed(int m, double *sp)
    {
        if (m < 0 || m >= m_numberOfJoints || !sp) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_MOTOR_ENCODER_SPEED, m, sp) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }


    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncoderSpeeds(double *spds)
    {
        if (!spds) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_MOTOR_ENCODER_SPEEDS, spds) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncoderAcceleration(int m, double *acc)
    {
        if (m < 0 || m >= m_numberOfJoints || !acc) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_MOTOR_ENCODER_ACCELERATION, m, acc) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }


    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getMotorEncoderAccelerations(double *accs)
    {
        if (!accs) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_MOTOR_ENCODER_ACCELERATIONS, accs) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }


    /* ITorqueControl */
    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getTorque(int j, double *t)
    {
        if (j < 0 || j >= m_numberOfJoints || !t) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_TRQ, j, t) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getTorques(double *t)
    {
        if (!t) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_TRQS, t) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    /* IPWMControl */
    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getNumberOfMotors(int *number)
    {
        if (!number) return ReturnValue_error_method_failed;
        *number = m_numberOfJoints;
        return ReturnValue_ok;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getDutyCycle(int m, double *val)
    {
        if (m < 0 || m >= m_numberOfJoints || !val) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_PWMCONTROL_PWM_OUTPUT, m, val) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getDutyCycles(double *vals)
    {
        if (!vals) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_PWMCONTROL_PWM_OUTPUTS, vals) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    /* IAmplifierControl */
    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getCurrent(int j, double* val)
    {
        if (j < 0 || j >= m_numberOfJoints || !val) return ReturnValue_error_method_failed;

        return getSingleData(VOCAB_AMP_CURRENT, j, val) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

    yarp::dev::ReturnValue ReadOnlyRemoteControlBoard::getCurrents(double *vals)
    {
        if (!vals) return ReturnValue_error_method_failed;

        return getVectorData(VOCAB_AMP_CURRENTS, vals) ? ReturnValue_ok : ReturnValue_error_method_failed;
    }

}
}
