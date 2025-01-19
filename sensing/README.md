# Sensing Functionality for Detecting Laser Spot on Target Laser Receiver Board

## Table of Contents
- [Installation](#installation)
- [Usage and Examples](#usage-and-examples)

## Installation

### Requirements
- OpenCV >= 4.10.0
- Python 3.11.2

### Method: From Source
1. Clone the repository:
   ```bash
   git clone https://github.com/LinF-PDX/capstone
   cd capstone/sensing
   pip install -r requirements.txt
   mkdir data
2. Setup CAN-HAT follow instruction in:
   https://www.waveshare.com/wiki/2-CH_CAN_HAT#Specifications
## Usage and Examples
### Run
    ```bash
    python sensing.py --surveydistance 100.0 --wheelbase 1300 --heightthreashold 10.0 --actualboardwidth 136 --lasercolor green --gpu 0
    ```
   surveydistance: Total distance the profileograph travel unit in m
   
   wheelbase: Distance between two side wheels for measurement unit in mm
   
   heightthreashold: The height difference threadshold that any larger value will be mark as wrong unit in mm
   
   actualboardwidth: The width of laser board in mm
   
   lasercolor: The color of laser default in green
   
   gpu: Enable gpu for image processing
   
### Aid
   ```bash
    python3 find_color_corner.py
   ```
   After running this script, clicking four corner of laser board captured to locate the board. Value will be used in backend.py variable roi.
