#ifndef __TEMPERATUREMANAGER__
#define __TEMPERATUREMANAGER__

#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/BufferedPort.h>

class TemperatureManager: public yarp::os::RFModule
{
private:
    yarp::dev::IMotor *_imot;
    yarp::os::BufferedPort<yarp::os::Bottle>  _outputPort;

    double *_motorTemperatures;
    double *_motorTemperatureLimits;
    int _nmotors;
    int _nEnabledMotors = 0;

    std::string _portPrefix="/5-setup";
    double _updatePeriod = 1; //seconds
    std::string _robotName= "icub";
    yarp::sig::Vector _listOfJoints = 0;

    yarp::dev::PolyDriver _motionControlDevice;

    bool sendData2OutputPort(double *temperatures);
    bool alloc(int nm);
    bool dealloc();

public:

    TemperatureManager();
    ~TemperatureManager() override;

    TemperatureManager(const TemperatureManager&) = default;
    TemperatureManager(TemperatureManager&&) = default;
    TemperatureManager& operator=(const TemperatureManager&) = default;
    TemperatureManager& operator=(TemperatureManager&&) = default;

    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();
};

#endif
