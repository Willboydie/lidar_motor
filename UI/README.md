## To create venv:
py -3.12 -m venv open3d_env

## To start the venv: 
.\open3d_env\Scripts\activate

## To install Open3d:
pip install open3d

## FOC realtime plotter
Plots the binary DualMotorStatusPacket stream (`Interface::sendDualMotorStatus`) in real time.
Select the COM port, pick a motor and tick the channels you want to see.
Firmware text output (Serial.println) appears in the log panel at the bottom.

pip install PySide6 pyqtgraph pyserial
python foc_plotter.py