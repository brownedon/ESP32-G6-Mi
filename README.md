# ESP32-G6-Mi
ESP32 to Dexcom G6 ->Amazfit Bip/MI Smartwatch
Allows an ESP32 to connect to a G6 and a Smartwatch.  Borrows heavily from Max Kaiser.
You need to put your transmitter id into the esp32_reader_mi.ino file.<br><pre>
//Dexcom Transmitter ID
static std::string transmitterID = "31H4DB";</pre>

You also need this to be false, until you've completed pairing with a watch<br>
<pre>
//MI Band MAC
//First time pairing control
boolean authenticated = true;
</pre>

Note: there's a watchdog timer in this code that will restart the device when it hangs.  
It will hang every 1-3 days.  I consider this to be an Arduino ESP32 implementation issue.<br>
You'll lose history and miss a couple of readings when this happens.

I would love to talk to someone who knows more about the bonding process.  
Some dexcom devices exchange an application key, and there doesn't appear to be a way to get the ESP32 Arduino to handle that.
I think newer transmitters are fine, it's older transmitters and "3" series transmitters that won't work.

Warning: this does all the crazy things that I like.  Predicts future glucose, doesn't send anything to the watch if you're solidly between 90 and 160.
