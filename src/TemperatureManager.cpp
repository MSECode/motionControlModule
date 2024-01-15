#include "TemperatureManager.h"

#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>

using namespace std;
using namespace yarp::os;


bool TemperatureManager::configure(yarp::os::ResourceFinder& rf)
{
    // Read configuration file
    Bottle &conf_group = rf.findGroup("GENERAL");
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        if(conf_group.check("portprefix")) { _portPrefix = conf_group.find("portprefix").asString(); }
        if(conf_group.check("period")) { _updatePeriod = conf_group.find("period").asFloat64(); }
        if(conf_group.check("robotname")) { _robotName = conf_group.find("robotname").asString(); }
    }
    
    // Create remote motion control device
    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+ _robotName + _portPrefix);
    options.put("local", _portPrefix + "/mc");


    yDebug() << "++++ config:" << _portPrefix;

    _motionControlDevice.open(options);

    if (!_motionControlDevice.isValid())
    {
        yError() << "Unable to open device driver. Aborting...";
        return false;
    }
    
    if (!_motionControlDevice.view(_imot) || _imot==nullptr)
    {
        yError() << "Unable to open motor raw interface. Aborting...";
        return false;
    }
    
    if (!_imot->getNumberOfMotors(&_nmotors))
    {
        yError() << "Unable to retrieve the number of motors";
        return false;
    }
    else
    {
        yDebug() << "Working with" << _nmotors << "motors";
    }
    
    // Allocate memory for pointers
    if (!alloc(_nmotors))
    {
        yError() << "Error allocating memory for pointers. Aborting...";
        return false;
    }
    
    
    // open the communication port towards motor controller module
    if(!_outputPort.open(_portPrefix +"/motor_temperatures:o"))
    {
        yError() << "Error opening output port for motor control";
        return false;
    }

    for (uint8_t i = 0; i < _nmotors; i++)
    {
        if (!_imot->getTemperatureLimit(i, &_motorTemperatureLimits[i]))
        {
            yError() << "Unable to get motor temperature Limits. Aborting...";
            //return false;
        }
        else
        {
            yDebug() << "Limit for motor#" << i << "value:" << _motorTemperatureLimits[i];
        }
        
    }

    return true;
}

bool TemperatureManager::close()
{
    // Closing port explicitely
    yInfo() << "Calling close functionality\n";

    // Deallocating memory for pointers
    if (!dealloc())
    {
        yError() << "Error deallocating memory for pointer. Failing...";
        return false;
    }
    
    return true;
}

double TemperatureManager::getPeriod()
{
    return _updatePeriod;
}

bool TemperatureManager::updateModule()
{
    
    for (size_t i = 0; i < _nmotors; i++)
    {
    	_motorTemperatures[i]= 0;
    }
    // yDebug() << "Temperature vector has size" << sizeof(_motorTemperatures);
    if (!_imot->getTemperatures(_motorTemperatures))
    {
        yError() << "Unable to get motor temperatures. Aborting...";
    }

    sendData2OutputPort(_motorTemperatures);
    
    return true;
}


TemperatureManager::TemperatureManager(): _imot(nullptr)
{;}

TemperatureManager::~TemperatureManager()
{;}

// Private methods
bool TemperatureManager::sendData2OutputPort(double * temperatures)
{
    static yarp::os::Stamp stamp;

    stamp.update();

    Bottle &b = _outputPort.prepare();
    _outputPort.setEnvelope(stamp);

    b.clear();

    b.addFloat64(stamp.getTime());
    for (size_t i = 0; i < _nmotors; i++)
    {
        b.addFloat64(temperatures[i]);
	    uint8_t allarm=0;
	if(temperatures[i] >= _motorTemperatureLimits[i])
		allarm=1;
	    b.addInt8(allarm);
    }
    _outputPort.write();
    
    
    return true;
}

bool TemperatureManager::alloc(int nm)
{
    _motorTemperatures = allocAndCheck<double>(nm);
    _motorTemperatureLimits = allocAndCheck<double>(nm);

    return true;

}

bool TemperatureManager::dealloc()
{
    checkAndDestroy(_motorTemperatures);
    checkAndDestroy(_motorTemperatureLimits);

    return true;
}
