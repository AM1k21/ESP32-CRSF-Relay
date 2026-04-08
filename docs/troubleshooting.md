# Troubleshooting

## RC LOST

| Possible cause | Action |
| --- | --- |
| Receiver not powered | Check receiver voltage rail |
| RX/TX swapped | Verify GPIO 33 from receiver TX and GPIO 32 to receiver RX |
| Bind mismatch | Confirm bind phrase alignment |
| Baud mismatch | Ensure receiver CRSF output is configured correctly |

## MODULE OFF

| Possible cause | Action |
| --- | --- |
| Module not powered | Use external supply sized for module load |
| PCB wiring issue | Check GPIO 16, 17, 25 path to translator board |
| Buffer IC solder issue | Inspect SN74HC125 joints and VCC/GND |
| Buck converter problem | Validate ESP32 supply under load |

## DRONE LOST

| Possible cause | Action |
| --- | --- |
| Aircraft out of range | Move closer or increase TX power safely |
| Bind mismatch on aircraft side | Verify phrase and profile alignment |
| Telemetry disabled | Enable telemetry in ELRS setup |

## Brownouts and Resets

| Possible cause | Action |
| --- | --- |
| Powering high-output module from USB | Move module to battery-fed input |
| Insufficient current budget | Provide supply with enough peak current |
| Overloaded regulator | Upgrade regulator and thermal headroom |

## Dashboard Not Loading

| Possible cause | Action |
| --- | --- |
| Not connected to AP | Connect to `Comm_Relay_Manager` |
| Wrong URL | Open `http://192.168.4.1` |
| Browser cache issues | Hard refresh or test incognito |
