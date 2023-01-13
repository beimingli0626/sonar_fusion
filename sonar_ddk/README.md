# sonar_ddk
This repository is to be cloned onto the ddk drone for reading sonar messages and publish as rostopic for later use.

### Usage
- Run `python3 sonar_node.py` to initialize a ros process, which processes raw sonar readings and publishes a Range message to rostopic named `sonar_topic`. You might need to change UART port for sonar sensor, the current hardware setup use `UART_J10`. 
- Use `./record.sh` to record topics for postprocessing and visualization. The bags are saved under `/sdcard/bags`.

### Command line notes
- Transfer the whole repo to docker accessible repo on ddk:
    `scp -r * root@dragonfly26:/home/root/sonar_ddk`
- Example for transfer bags back to your laptop:
    `scp -r /sdcard/bags edwinli@edwins-mbp:/Users/edwinli/Downloads`
- Start docker: `docker start voxl_noetic_docker`
- Stop docker: `docker stop voxl_noetic_docker`
- Enter docker bash: `docker exec -it voxl_noetic_docker bash`
- Quit docker bash: `exit`
- Before unplugging the power source, power off devices with: `poff`