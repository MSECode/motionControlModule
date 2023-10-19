# motionControlModule

This module is a working example for building a YARP device that exploits the iMotor YARP interfaces to interact with the motion control module.

## Module description

Specifically to this example, we are defining a `PolyDriver` device that interacts with the pointer to the `iMotor` yarp interface and prints the motor temperatures to an output port. Those temperatures value can be obtained exploiting the interface method `getTemperatures()` and saved to a c-array of doubles.

## General code architecture

Considering that a properly built yarp module should derive from the public interface `yarp::os::RFModule`, whose complete explanation is available at [this link](https://yarp.it/latest/thrift_tutorial_simple.html#thrift_tutorial_simple_module), the following methods should be overridden:

- configure()
- getPeriod()
- updateModule()
- close()

## Usage of the application module

In order to use this module, whose final target is to print at the port called: `/5-setup/motor_temperatures:o` the temperatures of the motors connected, the following steps should be done:

- create inside the project the `build` directory
- run the commands in order:
    
    ```
    cd build
    ccmake ../
    make
    make install
    ```
- supposed to have the correct setup running on the `yarpserver` sending data to state ports, this module can be run with the following command:
    ```
    ./build/TemperatureManager --from app/conf/config.ini 
    ```
    This will instantiate the aformentioned port, where with a frequence of `_updatePeriod` time, the following values will be streamed on the port:|
    ```
    timeStamp | temperature_motor_0 | ... | temperature_motor_N | boolean_limit_overcame_0 | ... | boolean_limit_overcame_N
    ```

