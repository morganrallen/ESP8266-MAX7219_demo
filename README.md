Very basic port of [wayoda/LedControl][lc].

Requires (included) [arduino_style][as] from [OLIMEX/ESP8266][omesp].
This was to simplify the port and should not be required to actually make this work.


Around [line 287](https://github.com/morganrallen/ESP8266-MAX7219_demo/blob/master/user/user_main.c#L287) the number of devices is defined. Change to suit your needs.

Building
--------
```
cd $PATH_TO/esp_iot_sdk_v0.9.5/
git clone https://github.com/morganrallen/ESP8266-MAX7219_demo.git
make COMPILE=gcc
```

Then flash how ever you normally do.

[lc]: https://github.com/wayoda/LedControl "wayoda/LedControl"
[as]: https://github.com/OLIMEX/ESP8266/tree/master/arduino_style "arduino_style library from ESP8266"
[omesp]: https://github.com/OLIMEX/ESP8266 "OLIMEX ESP8266 examples"
