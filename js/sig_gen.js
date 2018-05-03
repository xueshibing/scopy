#!/usr/bin/scopy -s

/* How to run this script:
 * scopy --script sig_gen.js
 */

var OUTPUT_FILENAME = "m2k_siggen_temp.csv"
var MSEC_BEFORE_STOP = 30
var NB_ITERATIONS = 2

function log(message) {
  fileIO.appendToFile(message, OUTPUT_FILENAME)
}

function run() {
    siggen.mode[0]=0
    siggen.mode[1]=0
    siggen.constant_volts[0]=4
    siggen.constant_volts[1]=4
    for  (i = 0; i < NB_ITERATIONS; i++) {
        var date = new Date()
        var time_stamp = date.toTimeString()
        log(time_stamp)
        siggen.running=true
        msleep(MSEC_BEFORE_STOP * 1000);
        var siggenData = "," + "sig gen temp" + "," + siggen.current_temp + '\n'
        log(siggenData);
        siggen.running=false
    }
}

function connectToUSB(host) {
    printToConsole("Connecting to " + host + "...")
    var success = launcher.connect(host)

    if (success)
        printToConsole("Connected!")
    else
        printToConsole("Failed to connect to: " + host + "!")

    return success;
}

function connect() {
    var usb_devs = launcher.usb_uri_list()

    var usb_uri = usb_devs[0]
    if (usb_uri) {
        var connected = connectToUSB(usb_uri)
        if (!connected)
            return false;
    } else {
        printToConsole("No usb devices available")
        return false;
    }

    return true;
}

function main() {
      if (!connect())
          return Error()

    // Clears previous data, if any
    fileIO.writeToFile("", OUTPUT_FILENAME)

    var date = new Date()
    var time_stamp = date.toTimeString()
    log(time_stamp + ", calib temp, " + calib.dac_calib_temp + "\n");
    run();

    launcher.disconnect()
}

main()
