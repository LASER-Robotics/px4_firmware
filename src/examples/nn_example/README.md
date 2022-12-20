# Control allocation in PX4 using Neural network

This app intends to replicate this [paper](https://ieeexplore.ieee.org/document/9536649), that implements an NN control allocation for a mambo parrot quadrotor. The paper uses matlab and simulink to implement the approach. For the PX4, the model is trained in Python using Keras, and its deployment in C++ uses the library fdeep.

## How to use

1) Perform an experimet and get the log. The log is only properly stored if the robot disarms. When experimenting in SITL, the log is stored as an ulg file in *~/.ros/sitl_uav1/log*
2) Use *ulog2csv* (or another method) to get the csv files from the log
3) Use [this colab](https://colab.research.google.com/drive/1x1v62d5vH4t86yUo85NaMjrW_wngyTUL?usp=sharing) to get the dataset. It uses the files:  
    - actuator_controls_0 (tau_r, tau_p, tau_y, T_z)
    - actuator_outputs_0 (w1, w2, w3, w4)
    - vehicle_local_position_0 (v_x, v_y, v_z)
    - vehicle_angular_velocity_0 (Omega_x, Omega_y, Omega_z)
4) Use [this colab](https://colab.research.google.com/drive/1l2LvvcQPKEvTFkvrn5yN9LBuI5YN_f72?usp=sharing) to train the model and save it as a .h5 file.
5) Install the [frugally deep library](https://github.com/Dobiasd/frugally-deep#requirements-and-installation)
6) Use the command
    - python3 keras_export/convert_model.py *path_to_h5_file* ~/.ros/sitl_uav1/fdeep_model.json
    
    to convert the h5 file to an approate format for fdeep use.
7) Compile this rep, and run nn_test in the PX4 console to run this APP.

The current version (15/12/2022) works in simulation.
