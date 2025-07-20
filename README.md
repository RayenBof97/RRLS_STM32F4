# Robust Recursive Least Squares Implementation on STM32F401

In this repository, I implemented the **Robust Recursive Least Squares (RLS)** algorithm on the **STM32F401** microcontroller using the STM32 HAL framework.

This implementation is based on the simulation and study conducted in the companion repository, which focuses on the RLS and Robust RLS algorithms applied to a simple **RC circuit**:

🔗 [Python Simulation Repository — Recursive and Robust RLS](https://github.com/RayenBof97/Robust_RLS_Python)

The algorithm estimates the coefficients of a discrete-time transfer function in real-time by analyzing the capacitor voltage response to a square wave input. This embedded implementation demonstrates the feasibility of running adaptive system identification directly on a microcontroller.

## Author

Rayen Bouafif
