Torque Vectoring y Traction Control:
Este submódulo contiene la traducción a C del algoritmo de control dinámico desarrollado en Python que he estado haciendo. El código está optimizado para ejecutarse en tiempo constante y actualmente está configurado para tracción trasera (RWD).

- ¿Cómo funciona la dinámica?
La gran diferencia de este sistema respecto al controlador antiguo es que entiende la física del coche en tiempo real. En lugar de limitarse a seguir ciegamente el volante, el Torque Vectoring calcula los límites térmicos y de fricción del neumático estimando la carga vertical dinámica, sumando tanto la transferencia de pesos geométrica como el downforce aerodinámico. Un solver (Augmented-Lagrangian QP) reparte el par asimétricamente respetando estos límites físicos para generar el momento de guiñada óptimo sin saturar la elipse de Kamm (Fx-Fy). Además, incluye protecciones mecánicas integradas como un Launch Control que pretensa los palieres en parado, y un sistema de rescate que apaga el diferencial de par si el piloto hace contravolante para salvar un trompo.

Justo después de calcular el reparto lateral, entra en juego un Control de Tracción predictivo y adaptativo cuyo objetivo es mantener el neumático en su punto máximo de agarre longitudinal. En base a la carga vertical estimada de las ruedas traseras y la estimación de la pista, el sistema calcula el slip ideal usando una versión simplificada de la Pacejka MF. Si el coche está tomando una curva muy fuerte el algoritmo reduce este límite dinámicamente para dejarle margen lateral al neumático y que el chasis no sobrevire.

Para actuar, medimos el slip real filtrado y si la rueda patina por encima del target calculado, un controlador PI entra en acción. La salida del PI pasa por una función softplus matemática que garantiza que el TC solo puede restar par, jamás añadirlo. Como capa de seguridad extra, un filtro inercial vigila la aceleración angular de la rueda ignorando la resonancia típica de 15Hz del chasis, pudiendo cortar el gas en milisegundos si la rueda salta en el aire. Todo esto se resuelve siempre después de que el solver de Torque Vectoring haya asignado el par a las ruedas izquierda y derecha.

- Requisitos de sensores:
Para que todo funcione correctamente, el archivo de interfaz necesita leer datos del coche a 200Hz. Es obligatorio mapear las aceleraciones longitudinal y lateral y el yaw rate de la IMU, la posición del acelerador y freno, el ángulo de la dirección y la velocidad angular individual de cada inversor trasero. Sin las RPM independientes de cada rueda, el control de tracción no puede calcular el slip ni aplicar los recortes asimétricos.

- Cómo activarlo en la ECU:
Toda la matemática del modelo es no depende del hardware y el único archivo que se comunica con la estructura TeR y el bus CAN es gp_interface.c. Para activar este proyecto en el coche no hay que tocar el código del TRQMANAGER, solo hay que reasignar los punteros de función de la pipeline de torque.

Ve al archivo TeR_TRQMANAGER.c de la ECU y busca la función donde se inicializan los parámetros. Allí basta con arrancar las memorias del modelo, asignar nuestro wrapper intermedio como el modo de conducción principal, y apagar el TC genérico heredado. Lo apagamos por completo porque nuestro modo gp_mode_intermediate ya incluye el control de tracción avanzado de Pacejka integrado en su propia física justo antes de enviar el par a los inversores.

**¡¡¡HE TENIDO QUE CAMBIAR EL ARCHIVO TeR_ECU-testing/SOFTWARE/TeR_ECU/TeR/Src/TeR_TRQMANAGER.c , NO TE OLVIDES DE CAMBIARLO PARA QUE FUNCIONE!!!!**

- Comando terminal C:
#include "gp_interface.h"

// Dentro de la inicialización de la ECU
gp_init();
DriveConfig.drivingMode = gp_mode_intermediate;
DriveConfig.tractionControl = tractionControlOFF;