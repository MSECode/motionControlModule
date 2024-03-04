#ifndef __TEMPERATUREMANAGER__
#define __TEMPERATUREMANAGER__

#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/BufferedPort.h>

#include <utility>
#include <vector>
#include <map>

class TemperatureManager: public yarp::os::RFModule
{
private:
    std::vector<yarp::dev::IMotor*> _iMotorDevices;
    yarp::os::BufferedPort<yarp::os::Bottle>  _outputPort;

    // double *_motorTemperatures;
    // double *_motorTemperatureLimits;
    std::vector<double> _motorTemperatures;
    std::vector<double> _motorTemperatureLimits;
    int _nsubparts = 0;
    int _nmotors = 0;

    double _updatePeriod = 1; //seconds
    std::string _robotName= "icub";
    yarp::sig::VectorOf<std::pair<const char*, yarp::sig::VectorOf<int>>> _mapOfJoints = 0;

    /**
     * List of remote_controlboard devices opened by the RemoteControlBoardRemapper device.
     */
    std::vector<yarp::dev::PolyDriver*> _remoteControlBoardDevices;

    bool sendData2OutputPort(double *temperatures);
    bool alloc(int ns);
    bool dealloc();

    void closeAllRemoteControlBoards();

    bool attachAll(const yarp::dev::PolyDriverList &polylist);

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
