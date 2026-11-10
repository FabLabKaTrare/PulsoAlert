# PulsoAlert — Contrato BLE GATT v1.1 (propuesto para validación)

**Estado:** implementado en firmware v1.5.1; pendiente de compilación, prueba física y adaptación de la web. Mantiene los UUID del contrato v1.0.

## Servicio
Nombre anunciado: `PulsoAlert`  
UUID: `f0a10000-7b4d-4c6a-9e20-3b8c8f100001`

| Característica | UUID | Propiedades | Formato |
|---|---|---|---|
| MEASUREMENT | `f0a10001-7b4d-4c6a-9e20-3b8c8f100001` | Read, Notify | **8 bytes** |
| BPM_THRESHOLD | `f0a10002-7b4d-4c6a-9e20-3b8c8f100001` | Read, Write | uint16 LE |
| VIBRATION_CYCLES | `f0a10003-7b4d-4c6a-9e20-3b8c8f100001` | Read, Write | uint8 |
| LED_DURATION | `f0a10004-7b4d-4c6a-9e20-3b8c8f100001` | Read, Write | uint16 LE, minutos |
| CONFIG_STATUS | `f0a10005-7b4d-4c6a-9e20-3b8c8f100001` | Read, Notify | 4 bytes |

## MEASUREMENT (8 bytes)
| Offset | Tipo | Descripción |
|---|---|---|
| 0–1 | uint16 LE | BPM; 0 = no válido |
| 2 | uint8 | Contacto: 0 no, 1 sí |
| 3 | uint8 | Batería porcentaje: 255 = desconocido (no calibrado) |
| 4 | uint8 | Alerta: 0 normal, 1 activa |
| 5 | uint8 | Medición: 0 esperando, 1 midiendo, 2 señal inválida |
| **6–7** | **uint16 LE** | **Voltaje real medido, milivoltios; 0 = no disponible** |

Ejemplo: 3920 mV se interpreta como `3.920 V`. No convertir voltaje a porcentaje sin calibración. Los seis primeros bytes mantienen el significado de v1.0.

## Configuración
| Característica | Rango | Inicial | NVS |
|---|---|---|---|
| BPM_THRESHOLD | 30–120 BPM | 50 | `pulsoalert/umbral` |
| VIBRATION_CYCLES | 1–5 | 3 | `pulsoalert/ciclos` |
| LED_DURATION | 1–10 min | 2 | `pulsoalert/ledmin` |

Los ajustes se cargan de NVS al arrancar. La escritura se valida, persiste y aplica solo tras éxito. En una alerta ya activa, la nueva duración del LED puede acortar o ampliar la ventana respecto a su inicio; el cambio de ciclos del motor se aplica a la máquina de estados en curso.

## CONFIG_STATUS (4 bytes)
- Byte 0: código `0` inicial, `1` guardado, `2` inválido, `3` error NVS.
- Byte 1: parámetro `0` ninguno, `1` umbral, `2` ciclos, `3` LED.
- Bytes 2–3: uint16 LE, valor **vigente**.

La web debe escribir **un parámetro por vez**, esperar `CONFIG_STATUS` y leer de nuevo el parámetro para comprobarlo. No existe transacción atómica entre parámetros.

## Adaptación requerida en PulsoAlert Web
- Aceptar `MEASUREMENT` de 8 bytes y leer `getUint16(6, true) / 1000` para mostrar voltios.
- Mantener el porcentaje como desconocido mientras el byte 3 sea 255.
- Habilitar escritura secuencial de configuración y verificar `CONFIG_STATUS`.
- Mantener diseño y simulador aprobados; no confundir datos simulados con datos reales.

**Seguridad:** prototipo educativo sin autenticación BLE; no es un dispositivo médico.
