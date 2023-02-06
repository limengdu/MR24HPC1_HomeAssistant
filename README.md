# XIAO ESP32C3 accesses Home Assistant via ESPHome service

<div align=center><img width = 700 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/78.jpg"/></div>

This article will guide you through the installation of the ESPHome service in your own Home Assistant environment. By using the WiFi function of the XIAO ESP32C3, you will be able to connect your XIAO to the Home Assistant as part of your home terminal in a very smooth way.

In addition, we will build a Home Assistant with human presence detection in combination with the most popular 24GHz mmWave Human Static Presence Module Lite.


## Getting Started

If you want to follow this tutorial through everything, you will need to prepare the following.

<table align="center">
	<tr>
	    <th>XIAO ESP32C3</th>
        <th>24GHz mmWave Human Static<br>Presence Module Lite</th>
	</tr>
    <tr>
        <td><div align=center><img width = 100 src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png"/></div></td>
        <td><div align=center><img width = 210 src="https://files.seeedstudio.com/wiki/Radar_MR24HPCB1/0.jpg"/></div></td>
    </tr>
	<tr>
        <td align = "center"><a href="https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html">Get One Now</a></td>
        <td align = "center"><a href="https://www.seeedstudio.com/24GHz-mmWave-Sensor-Human-Static-Presence-Module-Lite-p-5524.html">Get One Now</a></td>
	</tr>
</table>

The ultimate goal of this project is to deploy 24GHz mmWave Human Static Presence Module Lite in the Home Assistant.

We have written complete configuration files and libraries for the 24GHz mmWave Human Static Presence Module Lite to facilitate your rapid deployment of sensor to Home Assistant in this project.

The content of this tutorial will broadly go through the following steps.

1. [Select your Home Assistant environment](#select-your-home-assistant-environment)
2. [Install and cofigure ESPHome in Home Assistant](#install-and-configure-esphome-in-home-assistant)
3. [Configure the XIAO ESP32C3 and ESPHome connection](#configure-the-xiao-esp32c3-and-esphome-connection)
4. [Configure Home Assistant Panel](#configure-home-assistant-panel)

Of course, if you are interested in how the XIAO ESP32C3 uses Grove in the Home Assistant, you can read this chapter directly.

- [Connect Grove to Home Assistant using XIAO ESP32C3](#connect-grove-to-home-assistant-using-xiao-esp32c3)

## Select your Home Assistant environment

In this routine, we will not expand on how to install the Home Assistant environment, we will assume that you already have a working Home Assistant device.

If you wish to learn how to install Home Assistant, then you can refer to the [official tutorial](https://www.home-assistant.io/installation/). We strongly recommend that you install Home Assistant using an x86 device as this is the most user friendly way for you to install Home Assistant with Supervised.

<div align=center><img width = 600 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/77.png"/></div>

According to the table above, it is most appropriate to install **Home Assistant OS** and **Home Assistant Supervised**, which will take a lot of hassle off your hands. If you are running Home Assistant on Docker with OpenWRT (e.g. using LinkStar H68K), please don't worry, we will also provide you with a detailed reference on how to do this.

We have also written how to install Home Assistant for some of Seeed Studio products, please refer to them.

- [Getting Started with Home Assistant on ODYSSEY-X86](https://wiki.seeedstudio.com/ODYSSEY-X86-Home-Assistant/)
- [Getting Started with Home Assistant on reTerminal](https://wiki.seeedstudio.com/reTerminal-Home-Assistant/)
- [Getting Started with Home Assistant on LinkStar H68K/reRouter CM4](https://wiki.seeedstudio.com/h68k-ha-esphome/)

## Install ESPHome in Home Assistant

### Step 1. Install ESPHome

- **Scenario 1: ESPHome installation under Home Assistant OS (with Add-on Store)**

If you have the Home Assistant OS installed, it has an Add-on Store, which makes it much easier to install ESPHome.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/79.png"/></div>

In the Add-on Store, you can search for and install ESPHome.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/80.png"/></div>

- **Scenario 2: ESPHome installed under Home Assistant under OpenWRT Docker/Docker (without Add-on Store)**

As we are installing the Home Assistant Container, we cannot simply download the ESPHome service via the Add-on Store, so a compromise is needed.

We need to download the ESPHome image.

```
esphome/esphome:latest
```

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/17.png"/></div>

On the page where the container is created, we need to make some simple settings.
- Container Name: your container name
- Docker Image: choose just downloaded **esphome** image
- Network: choose **host** mode
- Environment Variables(-e): your environment variable

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/18.png"/></div>

Once you have filled in the above, save and apply. You will see that the Container has been created. You also need to start it.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/19.png"/></div>

In order to achieve the same effect as downloading ESPHome in Home Assistant, we need to modify the configuration file under Home Assistant.

Go to the Home Assistant Container.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/55.png"/></div>

We go to the terminal in Home Assistant.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/56.png"/></div>

Enter the following command in the terminal.

```sh
vi configuration.yaml
```

Add following content to the end of `configuration.yaml`.

```
# Example configuration.yaml entry
panel_iframe:
  esphome:
    title: "ESPHome"
    url: "http://192.168.100.1:6052"
    icon: mdi:chip
```

Exit the docker container by type ```exit``` in the Home Assistant Container shell. Once this is done, we restart the Home Assistant  container.

Create a new browser page, enter address `http://homeassistant:8123/` and enter your Home Assistant account and you will see ESPHome appear in the toolbar on the left.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/57.png"/></div>

## Configure the XIAO ESP32C3 and ESPHome connection

### Step 2. Hardware preparation

The goal of our tutorial is to be able to see the data information of the 24GHz mmWave Human Static Presence Module Lite in the Home Assistant dashboard.

Connect the device to the computer through the main board. The wiring diagram is shown in the table below.

<table align="center">
    <tr>
	    <td colspan="4"><div align=center><img width = 700 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/62.jpg"/></div></td></td>
	</tr>
	<tr>
	    <td align="center">XIAO ESP32C3</td>
	    <td align="center"></td>
        <td align="center">24GHz mmWave Human Static<br>Presence Module Lite</td>
	</tr>
	<tr>
	    <td align="center">5V</td>
	    <td align="center">--></td>
        <td align="center">5V</td>
	</tr>
	<tr>
	    <td align="center">GND</td>
	    <td align="center">--></td>
        <td align="center">GND</td>
	</tr>
    <tr>
	    <td align="center">RX</td>
	    <td align="center">--></td>
        <td align="center">D6</td>
	</tr>
    <tr>
	    <td align="center">TX</td>
	    <td align="center">--></td>
        <td align="center">D7</td>
	</tr>
</table>

### Step 3. Download the sensor library to your Home Assistant

- **Scenario 1: ESPHome installation under Home Assistant OS (with Add-on Store)**

In order to download some files from Home Assistant, we need to use the Terminal & SSH service.

Please ensure that you have enabled Advanced Mode.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/81.png"/></div>

Then you can search for the add-on **Terminal & SSH** in the Add-on Store, please install it.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/82.png"/></div>

Once the installation is complete, you will be able to see the Terminal tool in the left-hand toolbar, which you can access by clicking on it in the bash window.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/83.png"/></div>

Enter the following command in the terminal.

```
cd /config/esphome/
curl -o R24dvd.h https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/R24dvd.h
```

In this way, you have installed the dependencies needed for the sensors under the specified path in Home Assistant.

- **Scenario 2: ESPHome installed under Home Assistant under OpenWRT Docker/Docker (without Add-on Store)**


Click on the ESPHome Container.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/48.png"/></div>

We go to the terminal in ESPHome.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/49.png"/></div>

Enter the following command in the terminal.

```
curl -o R24dvd.h https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/R24dvd.h
```

Wait a few moments and the sensor library will be downloaded to the ESPHome Container root directory.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/50.png"/></div>


### Step 4. Keep the XIAO ESP32C3 and Home Assistant on the same LAN

I am sure that your Home Assistant has already done the work of getting into the network, for example by connecting to your device via a network cable. All you need to do then is simply to turn on a local network (e.g. WiFi) so that the XIAO ESP32C3 can also connect to this network.

I will use the LinkStar H68K as an example below. My aim is to get the XIAO connected to the LinkStar H68K's hotspot.

In the **Network** tab in OpenWRT, select **Wireless** --> **ADD**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/58.png"/></div>

For **Transmit Power** in **Device Configuration**, select **auto**.

For the **Interface Configuration** settings, please fill in the following instructions.

- General Setup
    - Mode: Depends on how LinkStar accesses the internet. If you are using a cable connection then select **Client**, if you are connected to WiFi then select **Access Point**.
    - ESSID: Enter the name of your WiFi, please try not to have spaces or special characters.
    - Network: check **lan**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/23.png"/></div>

- Wireless Security
    - Encryption: WPA2-PSK
    - Key: Enter the WiFi password you wish to set.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/24.png"/></div>


Once you have filled in the above information, click **Save and Apply** in the bottom right hand corner and wait a few moments for LinkStar to open a hotspot.

When no device is connected to this hotspot, it will be displayed as no signal.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/60.png"/></div>

All things considered, let's return to the Home Assistant page.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/61.png"/></div>

Click on the **NEW DEVICE**. Then click on **Continue**.

Under the new pop-up window, please enter the name of the application you wish to set up, as well as the name and password of the hotspot you have set up in LinkStar (Or your own WiFi). Make sure that the XIAO ESP32C3 and Home Assistant on the **same LAN**.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/25.png"/></div>

Then click **Next**.

In the device type, please select **ESP32-C3**.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/26.png"/></div>

Then click **Next**.

<span id="jump1">Click on the **Encryption key** and save it in a secure location, we will use this key in a later step.</span>

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/27.png"/></div>

Then click **SKIP**.

### Step 5. Change the XIAO ESP32C3 configuration yaml file

Then, we click on the device tab we just created, with the **EDIT** button in the bottom left corner.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/28.png"/></div>

Please note that we need to make changes to this yaml file. We have divided the content to be modified into two main parts, corresponding to one and two in the diagram below.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/62.png"/></div>

- In the **①** of the content, please do not change the device name except the one you have configured, please refer to the code below for the rest of the content.

<p style=":center"><a href="https://github.com/limengdu/MR24HPC1_HomeAssistant/blob/main/xiaoesp32c3-mr24hpc1_part1.yaml" target="_blank"><div align=center><img width = 300 src="https://files.seeedstudio.com/wiki/seeed_logo/github.png" /></div></a></p>

- In the **②** of the content, delete the words "captive_portal:" and replace with the following.

<p style=":center"><a href="https://github.com/limengdu/MR24HPC1_HomeAssistant/blob/main/xiaoesp32c3-mr24hpc1_part2.yaml" target="_blank"><div align=center><img width = 300 src="https://files.seeedstudio.com/wiki/seeed_logo/github.png" /></div></a></p>

Then, please click on the **Save** button in the top right corner.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/63.png"/></div>

### Step 6. Upload firmware to XIAO ESP32C3

- **Method 1: Compile and upload directly**

If you are using an x86 device and can see XIAO on the device port, then you can compile and upload the program to XIAO.

Connect XIAO to your device.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/ESPHome/49.png"/></div>

Click on the three dots in the bottom right corner of the device bar and select **Install**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/84.png"/></div>

Click **Plug into the computer running ESPHome Dashboard**.

<div align=center><img width = 500 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/85.png"/></div>

Select the connected port.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/ESPHome/18.png"/></div>

Now it will download all the necessary board packages and flash the ESPHome firmware into the XIAO ESP32C3. If the flashing is successful, you will see the following output.


<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/ESPHome/19.png"/></div>

If you are unable to find the port after connecting XIAO to your device, then you can try using the second method.

- **Method 2: Upload compiled firmware using the host**

Soft routes like LinkStar H68K do not support recognition of external MCU devices, we need to download the compiled firmware first and then upload the firmware via another PC.

Click on the **Install** button in the top right hand corner. Then select the last item **Manual download**.

<div align=center><img width = 500 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/30.png"/></div>

Select **Modern format**.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/31.png"/></div>

It will then take a long time to download and compile, so please be patient. Once everything is ready, the firmware will be automatically downloaded to your computer.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/33.png"/></div>

To upload the firmware to XIAO ESP32C3 there are couple options here we show 2 ways of doing it:

- Option 1: using the [ESPhome Web tool](https://web.esphome.io/?dashboard_install) to upload.

Make sure you have the right drivers installed. Below are the drivers for common chips used in ESP devices.

1. CP2102 drivers: [Windows & Mac](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)
2. CH342, CH343, CH9102 drivers: [Windowns](https://www.wch.cn/downloads/CH343SER_ZIP.html), [Mac](https://www.wch.cn/downloads/CH34XSER_MAC_ZIP.html)
3. CH340, CH341 drivers: [Windowns](https://www.wch.cn/downloads/CH341SER_ZIP.html), [Mac](https://www.wch.cn/downloads/CH341SER_MAC_ZIP.html)

Open the [ESPhome Web tool](https://web.esphome.io/?dashboard_install) with Chrome or Edge web browser.

Click **CONNECT**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/34.png"/></div>

select the XIAO ESP32 serial port in the popup window.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/64.png"/></div>

Click **INSTALL** and then select the `.bin` file downloaded from above steps.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/35.png"/></div>

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/38.png"/></div>


- Option 2: Using the [esphome-flasher tool](https://github.com/esphome/esphome-flasher).

If you are still unable to upload firmware using method one after installing the driver and changing browsers, then you can try using method two. Please refer to the official tutorial for specific installation methods and instructions.

!!!Tip
    If you wish to observe the log messages of the XIAO ESP32C3, you can also view them through the View Logs button of this software.
    <div align=center><img width = 500 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/41.png"/></div>

Once the upload is complete, you can disconnect the XIAO ESP32C3 from the PC (unless you have a need to view the logs) and simply power the XIAO separately.

If all goes well, the XIAO ESP32C3 will search for and connect to the WiFi you have set up for it. 

Just like me, I use LinkStar H68K's network. You can find it in the network options and see the IP address assigned to it by LinkStar H68K.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/42.png"/></div>

Normally, at this point in Home Assistant, the status of the device will also change from offline to online.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/65.png"/></div>

## Configure Home Assistant Panel

### Step 7. Connect to XIAO ESP32C3

If you do not have many Home Assistant devices on your LAN, Home Assistant can automatically search for and add your ESP devices to the Devices tab. You can see this device inside the **Devices & Services** tab in **Settings**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/66.png"/></div>

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/67.png"/></div>

If it does not automatically search, you can also connect to the XIAO ESP32C3 based on its IP address.

Click **ADD INTEGRSTION** and search for **esphome**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/43.png"/></div>

Then enter the IP address of the XIAO ESP32C3 with port number **6053**. Then click on **SUBMIT**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/44.png"/></div>

If the IP address and port number entered are correct, then you will see that you are asked to enter the Encryption key, which we asked to save in [step 4](#jump1).

Then click on **SUBMIT**.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/68.png"/></div>

At this point, the steps to add the device have been successfully completed.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/51.png"/></div>

### Step 8. 24GHz mmWave Module Lite functions overview

We come to the Home Assistant's overview tab. We give a general overview of the basic functions of the operating panel.

Firstly, there is the **Custom Infor output switch**. The icon on the left indicates that the information output is switched off, while the icon on the right indicates that the information output is switched on. If you want to see the sensor return information in real time, then you should have to click on the lightning bolt icon on the right.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/69.png"/></div>

At the bottom of the card is a real-time data display page. Here you can observe the millimetre wave changes within the monitored environment and adjust the parameters in the card according to your location.

<div align=center><img width = 400 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/70.png"/></div>

Due to space limitations, you can refer to the following two documents for more information on the use of the sensor and a more detailed explanation of the parameters.

- [24GHz mmWave Human Static Presence Module Lite Wiki](https://wiki.seeedstudio.com/Radar_MR24HPC1/)
- [24GHz mmWave Human Static Presence Module Lite User Manual](https://files.seeedstudio.com/wiki/mmWave-radar/24GHz_mmWave_Sensor-Human_Static_Presence_Module_Lite_User_Manual.pdf)
- [24GHz mmWave Human Static Presence Module Lite sensor Datasheet](https://files.seeedstudio.com/wiki/mmWave-radar/24GHz_mmWave_Sensor-Human_Static_Presence_Module_Lite_Datasheet.pdf)

### Step 9. Configure Home Assistant panel

If you find the default cards very boring and unfriendly for presenting data, Home Assistant offers a wide range of ready-made dashboards to choose from.

You can make your own dashboard to suit your preferences.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/73.png"/></div>

For example, the option to control the output of information is turned into a nice switch.

<div align=center><img width = 600 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/74.png"/></div>

Turning the speed of human movement into a visual dashboard display.

<div align=center><img width = 600 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/75.png"/></div>

This is what I came up with. It looks like it has the makings of a smart home control centre.

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/76.png"/></div>

So far, we have successfully concluded our tutorial content.

## Connect Grove to Home Assistant using XIAO ESP32C3

Of course, there is more to the XIAO ESP32C3 than just support for the 24GHz mmWave Human Static Presence Module Lite in Home Assistant, and you can find more tutorials for your own use in this document.

- [Connect Grove to Home Assistant using XIAO ESP32C3](https://wiki.seeedstudio.com/Connect-Grove-to-Home-Assistant-ESPHome/)

Get your creative juices flowing!

## Troubleshooting

**FAQ1: I got the following error while uploading firmware using ESPhome Web tool, how can I fix it?**

<div align=center><img width = 600 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/37.png"/></div>

> A: If this prompt appears while uploading, disconnect the XIAO ESP32C3 from the PC. Then, hold the BOOT BUTTON, connect the board to your PC while holding the BOOT button, and then release it to enter bootloader mode. At this point it is sufficient to reconnect and upload the firmware again.

**FAQ2: I can't install esphome flasher under Linux following the tutorial of esphome flasher?**

> A: When executing the following commands, you need to select your system version, otherwise an error will occur. For example, my computer is Ubuntu 22.04, then the command that should be executed is as follows.

```
sudo apt install python3

pip3 install -U \
    -f https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-22.04/ \
    wxPython

pip3 install esphomeflasher
```

**FAQ3: I filled in the correct WiFi and password, why don't I see the IP address of the XIAO ESP32C3?**

> A: When you encounter this problem, please check that the XIAO ESP32C3's antenna is connected in place. If the antenna is already connected, please make sure that the XIAO is not further than 3m from the LinkStar if possible (unless you have replaced the antenna with a more powerful one).
If you still do not see XIAO, you can use the [esphome flasher](https://github.com/esphome/esphome-flasher) software to check the XIAO log information and check the XIAO connection through the logs.
You can re-plug the xiao to let it try to search for WiFi and connect again.

**FAQ4: My XIAO ESP32C3 is connected to LinkStar's network, but why don't I see the sensor data refreshed?**

> A: When we encounter this problem, we need to use the logs to understand the exact reason why the sensor is not returning data. The situation that has been found to be likely to be encountered so far is a situation where the sensor is not responding, then its logs will look like this.

<div align=center><img width = 600 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/71.png"/></div>

> If you see a similar log, please double check the following three places.
> 1. Whether the sensor is supplied with 5V.
> 2. Are the RX and TX pins of the sensor connected correctly.
> 3. Disconnect only the 5V wire from the sensor to the XIAO and reconnect it to allow the sensor to be powered up again.

> Generally speaking, the third point solves this problem. A normal log flow for data transfer should look like this.

<div align=center><img width = 600 src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/72.png"/></div>

## Tech Support

Please do not hesitate to submit the issue into our [forum](https://forum.seeedstudio.com/).


<br /><p style="text-align:center"><a href="https://www.seeedstudio.com/act-4.html?utm_source=wiki&utm_medium=wikibanner&utm_campaign=newproducts" target="_blank"><img src="https://files.seeedstudio.com/wiki/Wiki_Banner/new_product.jpg" /></a></p>

