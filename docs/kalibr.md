# Kalibr

Es un toolbox de calibración de sensores desarrollado por el grupo ASL de ETH Zürich. Su objetivo principal es alinear matemáticamente sensores como:
* Cámaras
* IMUs
* Sistemas con múltiples cámaras o múltiples IMUs.

Se usa en:
* Visual-Intertial Odometry (VIO)
* SLAM
* Drones
* Robots móviles
* AR/VR

## Multi-Camera Calibration

### Intrinsic Calibration
Describe cómo ve el mundo una cámara:
* Distancia focal
* Centro óptico
* Distorsion (radial, tangencial, etc.)

## Extrinsic Calibration
Describe la posición y orientación de una cámara respecto a otra:
* Rotación (R)
* Traslación (t)


## Función de Kalibr
* Calibra todas las cámaras juntas
* Aunque no todas vean el mismo patrón
* Soporta muchos modelos de cámara (pinhole, fisheye, omnidireccionales)

# IMU

Un IMU mide:
* Aceleración (accelerometer)
* Velocidad angular (gyroscope)

Da información rápida, pero deriva con el tiempo

# Cámara + IMU
* Cámara: precisa | lenta | sin escala absoluta
* IMU: ruidosa | muy rápida | tiene gravedad

Juntas forman VIO

# ¿Qué calibra Kalibr?
Kalibr estima:
1. Extrínsecos CAM-IMU
* Rotación y traslación entre cámara e IMU
2. Desfase temporal
* Cámara e IMU no están sincronizados
3. Intrínsecos de la IMU
* Bias del acelerómetro
* Bias del giroscopio
* Factores de escala