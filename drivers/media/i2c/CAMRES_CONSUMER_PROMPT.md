# Camera consumer driver integration with oplus_camres

## Bus topology

The camera I2C bus (QUP wrap2 SE1) is physically shared between AP and
SEE (ADSP sensor subsystem).  The `oplus-camres` kernel module manages
ownership: default is SEE; AP consumers acquire/release via the API
below.  Camera sensors and VCM are children of the camres I2C adapter
in DT — they communicate normally once the bus is acquired.

## API (`include/linux/oplus-camres.h`)

```c
int  oplus_camres_acquire(struct device *consumer);   /* take bus from SEE */
int  oplus_camres_release(struct device *consumer);   /* return to SEE (refcounted) */
bool oplus_camres_ap_holds(void);                     /* AP owns the bus? */
bool oplus_camres_is_ready(void);                     /* SEE answered the camera handshake:
                                                         ADSP up AND adsprpcd serving.  camres
                                                         polls the attach after SNS registration
                                                         (SESSION_RETRY_MS, 1 s) and, once acked,
                                                         reprobes its unbound children + emits
                                                         CAMRES_READY=1 */
```

Stubs (no-op, true) when `CONFIG_OPLUS_CAMRES` is disabled.

## Wiring rules

| Driver hook | Call |
|---|---|
| probe (both drivers) | if `!is_ready()` → `-EPROBE_DEFER` (camres reprobes the child itself when ready; no userspace trigger needed) |
| power on / resume | `acquire(dev)` before any I2C |
| power off / suspend | `release(dev)` after last I2C |

Consumers: `drivers/media/i2c/smartsens-sc.c`, `drivers/media/i2c/dw9719.c`.
