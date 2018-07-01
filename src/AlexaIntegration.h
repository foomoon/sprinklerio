#include <fauxmoESP.h>

//typedef std::function<void(unsigned char, const char *, bool)> TStateFunction;

class wifiRelay : public fauxmoESP
{

  public:

    //void onMessage2(TStateFunction fn) { _callback = fn; }
    void addDevice2(int pin, const char * device_name)
    {
      // add devices and populate arrays with associated name and gpio
      addDevice(device_name);
      _name.push_back(device_name);
      _gpio.push_back(pin);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);

    };

    // Find pin associated with device name
    int getPin(const char * device_name_)
    {

      for (uint i=0; i != _name.size(); i++)
      {

        if (strcmp(_name.at(i) , device_name_)==0)
        {
          return _gpio.at(i);
        }

      }
      return -1;
    }

    // Reset all pin states to LOW
    void resetPins()
    {

      for (uint i=0; i != _name.size(); i++)
      {
        digitalWrite(_gpio.at(i), LOW);
      }

    }

  private:

    //TStateFunction _callback = NULL;
    std::vector<int> _gpio;
    std::vector<const char *> _name;

};
