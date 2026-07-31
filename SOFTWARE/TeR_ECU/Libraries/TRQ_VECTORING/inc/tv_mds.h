#ifndef TV_MDS_H
#define TV_MDS_H

#include "TeR_CAN.h"              /* pipeline integration point: trqMap_t/trq_t/TeR */
#include "v1_vehicle_dynamics.h"  /* pure core */

#ifdef __cplusplus
extern "C" {
#endif

trqMap_t trqVectoring(trq_t limit);
trqMap_t tractionControl(trqMap_t in);
void tv_reset_state(void);

#ifdef __cplusplus
}
#endif

#endif /* TV_MDS_H */