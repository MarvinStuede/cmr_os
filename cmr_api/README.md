# **cmr_api**: Libraries for basic social functionality
This package mainly contains a catkin library which can be included in other packages to run basic social robot functions. The library wraps action clients which then execute the functions by calling the action servers. Therefore it is necessary, that the corresponding action servers are executed. See the README in `cmr_driver` for further information on how to start the servers.

Currently included:
- Speech API to call Text-to-Speech actions, trigger conversations or send requests to NLP (Dialogflow)
- LED API to set specific emotions to the LED Panel or colors to LED stripes
- Ear API to move the ears to a specific position, to a trajectory or run predefined emotions
- ARM API to move the arms to a specific position

## How to use
If you want to use the libraries, firstly add `cmr_api` to dependencies in your `package.xml` and `CMakeLists.txt`. Also add the `${cmr_api_INCLUDE_DIRS}` variable to your include path in `CMakeLists.txt`.

In your code, add the headers
```c++
#include <cmr_api/speech_api.h>
#include <cmr_api/led_api.h>
#include <cmr_api/ear_api.h>
#include <cmr_api/arm_api.h>
```
Create objects
```c++
cmr_api::Speech speech;
cmr_api::LEDAPI led;
cmr_api::EarAPI ears;
cmr_api::ArmAPI arms;
ros::NodeHandle nh;
```
Initialize objects with a node handle
```c++
speech.init(nh)
led.init(nh);
ears.init(nh);
arms.init(nh);
```
Then do some stuff
```c++
speech.getTTSClient()->say("Now I will look sad for 5 seconds");
led.getPanel()->setExpression(cmr_api::stringToExpression("SAD"), 5.0)
speech.getTTSClient()->say("Now I will wiggle my left ear for 3 seconds");
ears.motionWiggle(3.0, cmr_msgs::LimbNum::EAR_LEFT)
speech.getTTSClient()->say("Now I will move my right arm to 0.5 rad");
arms.moveToPos(0.5, cmr_msgs::LimbNum::ARM_RIGHT)
```
For further information, check the respective header files
