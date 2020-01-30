# MATLAB ET-DDF

This section of the ET-DDF repository contains the Matlab implementation of ET-DDF. **Note:** This implementation has not been updated since the v1.0 version, which was used to generate the results for the FUSION paper.

To run the Matlab code, use:

```matlab
fci_main
```

The simulation configuration options are at the beginning of the script, including *network connections*, event-triggering threshold *$$\delta$$*, and covariance intersection threshold *$$\tau$$*.

The **Agent** class functions as the message handling layer between the simulation and the **ETKF** event-triggered Kalman filter class.

To analyze simulation results use

```matlab
fci_plotting -- generate time trace plots
mse_plots -- generate mean-squared error plots
comms_data_post -- generate communications usage, message sharing, and CI trigger plots
```

Vehicle dynamics and filters are currently all linear, with plans to implement nonlinear dynamics, measurements and filters in the python version only.