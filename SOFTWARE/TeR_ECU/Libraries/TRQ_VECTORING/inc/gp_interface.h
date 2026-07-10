/*
 * gp_interface.h
 * Wrapper de integración entre TV y TeR_TRQMANAGER
 */

#ifndef GP_INTERFACE_H
#define GP_INTERFACE_H

#include "TeR_TRQMANAGER.h"

// Inicializa las memorias y estados estáticos
void gp_init(void);

// Función principal compatible con la estructura trqPipeline_t (*drivingMode)
trqMap_t gp_mode_intermediate(trq_t limit);

#endif // GP_INTERFACE_H