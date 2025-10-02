#ifndef __SENSOR_CFG_INPUT_H__
#define __SENSOR_CFG_INPUT_H__


#include <stdint.h>
#include <stddef.h>
#include "sensor_cfg_types.h"

/* ===== Return codes ===== */
enum {
    SENSOR_CFG_OK          =  0,
    SENSOR_CFG_ERR_CAP     = -1,  /* output capacity insufficient */
    SENSOR_CFG_ERR_PARSE   = -2,  /* input parse error */
    SENSOR_CFG_ABORTED     = -3   /* user chose to cancel/exit without return */
};

/* ===== Config (override if needed) ===== */
#ifndef UART_BASEADDR
#define UART_BASEADDR XPAR_XUARTPS_0_BASEADDR
#endif

#ifndef SENSOR_CFG_MAX_OVERRIDES
#define SENSOR_CFG_MAX_OVERRIDES 256
#endif

#ifndef SENSOR_CFG_WORKBUF_CAP
#define SENSOR_CFG_WORKBUF_CAP   1024
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Interactive UART tool.
 * - defaults/defaults_len : base configuration (read-only)
 * - out/out_cap           : buffer to receive the merged final array
 * - out_len               : number of valid entries in 'out' on success
 *
 * The tool accepts commands via UART, lets the user add/delete/list/validate,
 * and when the user types 'return'/'done'/'ok', it merges defaults+overrides
 * and returns them via out/out_len. It never writes the sensor.
 *
 * Returns:
 *   SENSOR_CFG_OK        on success (out/out_len valid)
 *   SENSOR_CFG_ERR_CAP   if out_cap insufficient for merged result
 *   SENSOR_CFG_ERR_PARSE if unrecoverable parse error (rare)
 *   SENSOR_CFG_ABORTED   if user typed 'cancel'/'exit'
 */
int sensor_cfg_input(const regval_list *defaults, size_t defaults_len,
                     regval_list *out, size_t out_cap, size_t *out_len);

#define ARRAY_LEN(a) (sizeof(a)/sizeof((a)[0]))

#ifdef __cplusplus
}
#endif


#endif
