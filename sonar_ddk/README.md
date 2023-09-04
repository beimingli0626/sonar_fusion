# sonar_ddk
This repository is to be cloned onto the drone for reading sonar messages and publish as rostopic for later use.

### Usage
- Run `python3 sonar_node.py` to initialize a ros process, which processes raw sonar readings and publishes a Range message to rostopic named `sonar_topic`. You might need to change UART port for sonar sensor, the current hardware setup use `UART_J10`. 
- Use `./record.sh` to record topics for postprocessing and visualization. The bags are saved under `/sdcard/bags`.