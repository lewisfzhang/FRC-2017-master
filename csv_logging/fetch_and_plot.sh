#!/bin/bash

scp admin@roborio-252-frc.local:/home/lvuser/SHOOTER-LOGS-JRAD.csv . && python plot_data.py 

