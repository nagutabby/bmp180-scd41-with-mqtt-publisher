# bmp180-scd41-with-mqtt
## Usage
Create `pyboard` directory in your home directory.

Download [pyboard.py](
https://github.com/micropython/micropython/blob/master/tools/pyboard.py) in the `pyboard` directory.

Run the following command to add execute permissions to `pyboard.py`:

```bash
sudo chmod a+x ~/pyboard/pyboard.py
```

Write the following command on `~/.bashrc` to add `pyboard.py` to PATH:

```bash
export PATH=~/pyboard:$PATH
```

Run the following command to load `~/.bashrc`.

```bash
source ~/.bashrc
```

Create `config.py` that following information is written:

```python
WIFI_SSID = <WIFI_SSID>
WIFI_PASSWORD = <WIFI_PASSWORD>
SERVER_MQTT_BROKER = <SERVER_ADDRESS_OF_MQTT_BROKER>
````

Run the following command to put `config.py` ESP32.

```bash
pyboard.py -d /dev/ttyUSB0 -f cp config.py :
```

Run the following command to install `mqtt.simple` library.

```bash
micropython -m mip install umqtt.simple
```

Run the following command to run `main.py` on ESP32.

```bash
pyboard.py -d /dev/ttyUSB0 main.py
```
