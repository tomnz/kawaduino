# Kawaduino #

This is an Arduino project that uses an external IC to query the onboard ECU of a Kawasaki motorcycle. Various data can be read back from the bike, including RPM, speed, throttle position, and sensor data.

### Application

This particular script is designed to power a string of AdaFruit NeoPixel LEDs like [these ones](http://www.adafruit.com/products/1461). In its current state, the script has two modes - one that is based on RPM (animates color), and one based on speed (animates moving dots).

### Details

A few miscellaneous points about the script and hardware:

* If the link with the ECU is broken for any reason (e.g. switching off the Arduino, and back on again), it will not accept a new connection for 10 seconds or so - in this time the LEDs will not do anything... Just give it a minute
* You'll notice when you switch modes that it blips an animation over the LEDs for a split second. The reason for this is convoluted, but interesting... The rate at which the ECU can be queried maxes out at around twice per second - so if you were to update the LEDs after each query, then the animation would be extremely choppy. However, the querying process is fairly sparse - there are a lot of "waits" in order to meet the requirements of the communication protocol. This gives an opportunity to update the LEDs during the communication downtime. I originally tried to achieve this using interrupts, but occasionally the timing would be thrown off by an LED update running slightly too long, and the communication would be cut off sporadically. Instead of relying on the hardware to do timing, I decided to implement my own "wait" function that would just cycle the LEDs continuously until the required time had passed - but this requires knowing how long each animation cycle takes in order to not overshoot the timing. Hence, whenever switching modes (or turning on the Arduino), the animation runs a bunch of cycles to determine the average refresh time. This is what causes the "blip".
* Because we are animating often, but updating the variables from the ECU only occasionally, some work is needed to "smooth" out the changes in raw data... This means the animations look much nicer, but at the cost of introducing some lag time between receiving a value from the ECU, and the LEDs reflecting that value

### Hardware

There are several types of chips that can facilitate communication with the ECU. One example of a chip that works is the [MC33660 SOIC](http://www.digikey.com/product-detail/en/MC33660EFR2/MC33660EFR2CT-ND/5215177). I'd recommend picking up an SOIC to DIP adapter off eBay to make working with the chip easier. Be sure to check the data sheet for the chip that you choose - many will benefit from external caps/resistors to provide protection from voltage spikes.

Obviously you will also need an LED string. I'd recommend a button to switch modes - and possibly a switch to disable the whole system when you need to. A rough overview of my entire installation:

* I have a 12V -> 5V DC converter that ultimately powers both the LEDs and the Arduino
* A relay switches power to the converter on/off when the bike is powered on, meaning no dead battery!
* A physical switch under the tail of the bike can break the relay on signal - allowing me to kill power with the switch
* A physical button under the tail lets me switch between the modes
* The IC and various caps/resistors are soldered onto a prototyping shield, and wrapped up in a makeshift container with the Arduino
* Wires with pins soldered on the end run to the ECU port (I was unable to source the male plug)
* Two LED strings are snaked through the internals of the bike, lighting up the engine and wheels