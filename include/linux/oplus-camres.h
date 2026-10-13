/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _OPLUS_CAMRES_H
#define _OPLUS_CAMRES_H

struct device;

#if IS_ENABLED(CONFIG_OPLUS_CAMRES)
int oplus_camres_acquire(struct device *consumer);
int oplus_camres_release(struct device *consumer);
bool oplus_camres_ap_holds(void);
bool oplus_camres_is_ready(void);
#else
static inline int oplus_camres_acquire(struct device *c) { return 0; }
static inline int oplus_camres_release(struct device *c) { return 0; }
static inline bool oplus_camres_ap_holds(void) { return true; }
static inline bool oplus_camres_is_ready(void) { return true; }
#endif

#endif
