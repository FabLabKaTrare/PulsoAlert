# PulsoAlert Firmware v1.5.1

## Archivos
- `PulsoAlert_v1.5.1.ino`: firmware basado en v1.5.0 verificado con BLE.
- `PulsoAlert_Contrato_BLE_v1.1.md`: extensión del contrato con voltaje real.

## Instalación
1. Abrir `.ino` en Arduino IDE y seleccionar la misma placa ESP32-C3 usada en v1.5.0.
2. Conservar las bibliotecas que ya funcionan: U8g2, SparkFun MAX3010x, BLE incluida en el core ESP32.
3. `Preferences` y FreeRTOS forman parte del core Arduino-ESP32.
4. Activar `USB CDC On Boot = Enabled` para la consola.
5. Compilar y cargar; abrir monitor serie a 115200.

## Pruebas sugeridas
1. Verificar OLED, botón, MAX30102, motor y LED.
2. Conectar por BLE; comprobar BPM, contacto y alertas.
3. Verificar `MEASUREMENT` de 8 bytes y el voltaje en milivoltios.
4. Escribir umbral=55, ciclos=2 y LED=3 (secuencialmente). Confirmar `CONFIG_STATUS=1` y lectura posterior.
5. Reiniciar la placa y verificar que NVS conserva 55/2/3.
6. Escribir un valor fuera de rango, comprobar `CONFIG_STATUS=2` y que el valor anterior permanece.
7. Probar desconexión y reconexión.

