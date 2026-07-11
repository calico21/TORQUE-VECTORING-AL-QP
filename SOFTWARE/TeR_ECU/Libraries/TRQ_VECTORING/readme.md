TRQ_VECTORING V2.0
Este submódulo contiene la traducción a C puro de la arquitectura matemática de control dinámico que estuve desarrollando en Python. El código está hiper-optimizado para ejecutarse con latencia determinista en el microcontrolador y actualmente está configurado en exclusiva para tracción trasera (RWD).

¿Cómo funciona la dinámica de verdad? (Torque Vectoring)La gran diferencia de este sistema respecto al controlador antiguo es que entiende la física del coche en tiempo real. En lugar de limitarse a seguir ciegamente el ángulo del volante con un PID guarro, el Torque Vectoring calcula los límites térmicos y de fricción absolutos de cada neumático. Lo hace estimando la carga vertical dinámica, sumando tanto la transferencia de pesos geométrica instantánea como el downforce aerodinámico en función de la velocidad.

Con esas cajas de límites calculadas, el cerebro del sistema es un solver Augmented-Lagrangian QP (AL-QP) de 16 iteraciones. Este solver matemático reparte el par de forma asimétrica (izquierda/derecha) respetando estrictamente los límites físicos para generar el momento de guiñada (M_z) óptimo. Si un neumático pisa hielo o se sobrecalienta el inversor, entra en juego el Friction Budgeting: el solver ahoga automáticamente la demanda total del piloto antes de desestabilizar el chasis, evitando el mítico "efecto péndulo" y salvándonos de un trompo instantáneo.

Además, incluye protecciones mecánicas integradas: un Launch Control que pretensa los palieres en parado para no partirlos, y un gate de oversteer que mata el diferencial asimétrico si el piloto tiene que hacer contravolante para salvar el coche. Y ojo, ya no asumimos que el coche va sobre raíles: el código integra un Observador de Velocidad Lateral (v_y) por Fading-Memory que usa la IMU para calcular la deriva real del chasis.

Tracción Predictiva Pacejka (Control de Tracción)
Justo después de que el AL-QP calcule el reparto lateral perfecto, entra el TC. Pero no es un TC reactivo de calle, es predictivo.

Integramos un estimador RLS (Recursive Least Squares) emparejado con un método secante que calcula en vivo la derivada de la curva de Pacejka (delta_F_x/delta_sr). Básicamente, el algoritmo va "cazando" la cima del agarre longitudinal. Si el asfalto cambia, el target de slip se adapta dinámicamente antes de que la rueda siquiera empiece a patinar descontroladamente. Si estamos tomando una curva a fondo, la elipse de Kamm entra en juego y el sistema recorta este límite de slip para dejar margen de agarre lateral y que no perdamos el tren trasero.

Para actuar, medimos el slip real filtrado y, si nos pasamos del target, entra un controlador PI. Detalle clave: La salida de este PI pasa por una función matemática softplus que garantiza por diseño que el TC solo puede restar par, jamás añadirlo. Como red de seguridad final contra vibraciones, un filtro inercial vigila la derivada de la velocidad de la rueda para cortar el gas en milisegundos si saltamos por un piano, ignorando la resonancia natural de 15Hz de la transmisión.

Requisitos de SensoresPara que esta matemática no explote, el archivo de interfaz (gp_interface.c) necesita tragar datos limpios del bus a 200Hz. Es estrictamente obligatorio mapear:Aceleración longitudinal (a_x) y lateral (a_y).Yaw rate de la IMU (w_z).Posición de pedales (acelerador y presión de freno).Ángulo de la dirección (steering).Velocidad angular en bruto de CADA inversor trasero (sin las RPM individuales, el RLS es ciego y el TC no hace nada).Temperaturas de los inversores (para el derating térmico).Telemetría Avanzada (El Muro de Boxes)El código empaqueta la física interna en 3 tramas CAN dedicadas para que no vayamos a ciegas cuando lo subamos a los caballetes o lo metamos en pista. Se escupen por el bus y se pueden leer con nuestro dashboard de PyQtGraph:0x100 (Dinámica): Deriva lateral (v_y), residuo del solver KKT, y targets de slip óptimo.0x101 (Estimador RLS): Las derivadas de Pacejka (theta) y la superficie de fricción estimada (mu).0x102 (Actuadores): Par físico demandado a los inversores y slip filtrado actual.

⚠️ CÓMO ACTIVARLO EN LA ECU (¡LEER!) ⚠️Toda la algoritmia es 100% agnóstica al hardware. El único puente con el bus CAN y la estructura de TeR es gp_interface.c. No hay que tocar ni una línea del código del TRQMANAGER heredado, solo reasignar los punteros.

¡¡¡HE TENIDO QUE CAMBIAR EL ARCHIVO TeR_ECU-testing/SOFTWARE/TeR_ECU/TeR/Src/TeR_TRQMANAGER.c , NO TE OLVIDES DE CAMBIARLO PARA QUE FUNCIONE!!!!Ve a la inicialización de parámetros de la ECU y arranca nuestro wrapper como modo principal. Hay que APAGAR el TC antiguo genérico por completo porque nuestro modo gp_mode_intermediate ya inyecta el TC de Pacejka justo después del Torque Vectoring.C

#include "gp_interface.h"

// Dentro de la inicialización de la ECU:
gp_init(); 
DriveConfig.drivingMode = gp_mode_intermediate; 
DriveConfig.tractionControl = tractionControlOFF; // APAGADO. La lógica va por dentro delmodo