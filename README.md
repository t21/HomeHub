# HomeHub

### Command set

#### General commands

| AT-command | Description | Response(s) |
|------------|-------------|-------------|
| AT         | Command that can be used to check if the communication link is working. | OK |
| ATZ        | Command that resets the BLE-module | OK |
| AT+INFO?   | Command that returns information of the BLE-module. | +INFO: HW:2.1, FW:xx, SD:yy, OK |


#### Scan commands

| AT-command | Description | Response(s) |
|------------|-------------|-------------|
| AT+SCANSTART | Command that starts the scanning for registered devices. | +SCANSTART: OK - scanning was successfully started. <BR> +SCANSTART: ERROR - scanning was NOT successfully started. |
| AT+SCANSTOP | Command that stops the scanning for devices. | +SCANSTOP: OK - scanning was successfully stopped. <BR> +SCANSTOP: ERROR - scanning was NOT successfully stopped. |
| AT+SCANSETP=&lt;active>,&lt;selective>,&lt;interval>,&lt;window>,&lt;timeout> | Command that sets the scan parameters. <BR> &lt;active> - '0' no scan response, '1' request scan response <BR> &lt;selective - '0' scan for all devices, '1' only scan for whitelisted devices <BR> &lt;interval> - scan interval between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s) <BR> &lt;window> - Scan window between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s) <BR> &lt;timeout> -  Scan timeout between 0x0001 and 0xFFFF in seconds, 0x0000 disables timeout | +SCANSETP: OK <BR> +SCANSETP: ERROR |


#### Device handling commands

| AT-command | Description | Response(s) |
|------------|-------------|-------------|
| AT+DEVADD=&lt;AA>:&lt;BB>:&lt;CC>:&lt;DD>:&lt;EE>:&lt;FF>  | Command that lets you set the Bluetooth Device Address of a device that you would like to scan for. The device is added at the end of the list. | +DEVADD: OK - the device was successfully added <BR> +DEVADD: ERROR - the device was NOT successfully added |
| AT+DEVLIST=&lt;item nbr>? | Command that lets you fetch information on the device with list id &lt;item nbr> | |
