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
    Bottle* jointsBottle = nullptr;
    Bottle* subpartsBottle = nullptr;
    std::vector<std::string> localPortPrefixes;
    std::vector<std::string> remoteControlBoardsPorts;
    if (conf_group.isNull())
    {
        yWarning() << "Missing GENERAL group! The module uses the default values";
    }
    else
    {
        if(conf_group.check("robotname")) { _robotName = conf_group.find("robotname").asString(); }
        if(conf_group.check("period")) { _updatePeriod = conf_group.find("period").asFloat64(); }
        if (conf_group.check("listofsubparts") && conf_group.check("listofjoints") )
        {
            subpartsBottle = conf_group.find("listofsubparts").asList();
            jointsBottle = conf_group.find("listofjoints").asList();
            _nsubparts = jointsBottle->size();
            remoteControlBoardsPorts.resize(_nsubparts);
            localPortPrefixes.resize(_nsubparts);

            if(subpartsBottle->size() != jointsBottle->size())
            {
                yError() << "Dimension of subparts and joints lists must be equal";
                return false;
            }
            else
            {
                for(int i=0; i < _nsubparts; i++) 
                {
                    localPortPrefixes.push_back("/" + subpartsBottle->get(i).asString() + "/mc");
                    remoteControlBoardsPorts.push_back("/"+ _robotName + "/" + subpartsBottle->get(i).asString());
                    for (int j = 0; j < jointsBottle->get(i).asList()->size(); j++)
                    {
                        _mapOfJoints.push_back({subpartsBottle->get(i).asString().c_str(), jointsBottle->get(i).asList()->get(j).asInt32()});
                        ++_nmotors;
                    }
                    yDebug() << "Inserted element: <" << subpartsBottle->get(i).asString().c_str() << "," << jointsBottle->get(i).asList()->toString() << ">";
                }
            }
        }
    }
    
    yDebug() << "++++ config ++++:\n" 
    << "\t period: " << _updatePeriod << "\n"
    << "\t robotname: " << _robotName << "\n"
    << "\t listsubparts: " << subpartsBottle->toString() << "\n"
    << "\t listofjoints: " << jointsBottle->toString() << "\n";
    
    // Create remote motion control devices (one per each subparts)
    // Parameters loaded, open all the remote controlboards
    _remoteControlBoardDevices.resize(remoteControlBoardsPorts.size(), nullptr);
    yarp::dev::PolyDriverList remoteControlBoardsList;

    for (uint8_t i = 0; i < remoteControlBoardsPorts.size(); i++)
    {
        yarp::os::Property options;
        options.put("device", "remote_controlboards");
        options.put("local", localPortPrefixes[i]);
        options.put("remote", remoteControlBoardsPorts[i]);
        
        _remoteControlBoardDevices[i] = new yarp::dev::PolyDriver();
        bool ok = _remoteControlBoardDevices[i]->open(options);
        if(!ok || !(_remoteControlBoardDevices[i]->isValid()))
        {
            yError() << "Unable to open device driver #:" << i << "Closing devices...";
            closeAllRemoteControlBoards();
            return false;
        }

        std::string res = ok ? "TODO BIEN" : "SKIFOOO";
        yDebug() << res.c_str() << "\n";

        // We use the remote name of the remote_controlboard as the key for it, in absence of anything better
        remoteControlBoardsList.push((_remoteControlBoardDevices[i]),remoteControlBoardsPorts[i].c_str());
    }
    
    if(!attachAll(remoteControlBoardsList))
    {
        yError() << "AttachAll failed, some subdevice was not found or its attach failed";
        return false;
    }
    yDebug() << "Working with totally" << _nmotors << "motors and" << _nsubparts << "subparts";
    
    // Allocate memory for pointer
    if (!alloc(_nsubparts))
    {
        yError() << "Error allocating memory for pointers. Aborting...";
        return false;
    }
    
    for (uint8_t i = 0; i < _mapOfJoints.size(); i++)
    {
        yarp::dev::IMotor *imotp = _iMotorDevices.at(i);
        if (!imotp->getTemperatureLimit(i, &_motorTemperatureLimits[i]))
        {
            yError() << "Unable to get motor temperature Limits. Aborting...";
            return false;
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

    closeAllRemoteControlBoards();
    
    return true;
}

double TemperatureManager::getPeriod()
{
    return _updatePeriod;
}

bool TemperatureManager::updateModule()
{
    int jointNib = 0;
    for (int i = 0; i < _mapOfJoints.size(); i++)
    {
        yarp::dev::IMotor *imotp = _iMotorDevices.at(i);
    	_motorTemperatures[i]= 0;
        jointNib = i;
        if (!imotp->getTemperature(jointNib, &_motorTemperatures[jointNib]))
        {
            yError() << "Unable to get motor " << jointNib << " temperature.\n";
        }
    }

    sendData2OutputPort(&_motorTemperatures[jointNib]);
    
    return false;
}


TemperatureManager::TemperatureManager()
{;}

TemperatureManager::~TemperatureManager()
{;}

// // Private methods
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

bool TemperatureManager::alloc(int ns)
{
    
    _motorTemperatures.resize(ns);
    _motorTemperatureLimits.resize(ns);

    return true;

}

bool TemperatureManager::dealloc()
{
    _motorTemperatures.resize(0);
    _motorTemperatureLimits.resize(0);
    

    return true;
}

void TemperatureManager::closeAllRemoteControlBoards()
{
    for(auto& _remoteControlBoardDevice : _remoteControlBoardDevices)
    {
        if( _remoteControlBoardDevice )
        {
            _remoteControlBoardDevice->close();
            delete _remoteControlBoardDevice;
            _remoteControlBoardDevice = nullptr;
        }
    }

    _remoteControlBoardDevices.resize(0);
}

bool TemperatureManager::attachAll(const yarp::dev::PolyDriverList &polylist)
{
    // call attach all
    for (uint8_t i = 0; i < polylist.size(); i++)
    {    
        yarp::dev::IMotor *imotp = _iMotorDevices.at(i);
        if (!polylist[i]->poly->view(imotp) || imotp==nullptr)
        {
            yError() << "Unable to open motor raw interface. Aborting...";
            return false;
        }
        // open the communication port towards motor controller module
        yDebug() << "Requesting opening of port" << (_robotName+"/motor_temperatures:o");
        if(!_outputPort.open(_robotName+"/motor_temperatures:o"))
        {
            yError() << "Error opening output port for motor control";
            return false;
        }
    }
    return true;
}