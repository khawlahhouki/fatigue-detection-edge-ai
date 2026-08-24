#ifndef INC_DEBUG_CONFIG_H_
#define INC_DEBUG_CONFIG_H_

#include "safe_printf.h"

/* ============================================================================
 * INTERRUPTEUR DE DEBUG PAR TACHE
 * ============================================================================
 * Mets 1 UNIQUEMENT sur la tache que tu veux observer en ce moment, 0
 * partout ailleurs. Comme ce sont des macros (valeur connue a la
 * compilation), le compilateur ELIMINE ENTIEREMENT le code du printf
 * quand le flag vaut 0 -- pas juste "on saute l'appel a l'execution",
 * le SafePrintf() n'existe carrement plus dans le binaire final pour
 * cette tache. Cout CPU reel : ZERO, garanti, pas juste "tres faible".
 *
 * C'est la difference cle avec le throttling (dsp_dbg_counter % 5, etc.) :
 * le throttling reduit le risque, ceci l'ELIMINE par construction pour
 * toute tache mise a 0.
 * ============================================================================ */

#define DEBUG_EMG        0
#define DEBUG_PPG        0
#define DEBUG_IMU        0
#define DEBUG_TEMP       0
#define DEBUG_DSP        1  /* <-- exemple : seule StartTaskDSP parle en ce moment */
#define DEBUG_TEMPPROC   0
#define DEBUG_FEATURES   1
#define DEBUG_IMUPROC    0
#define DEBUG_LORA       0

/* ---- Macro generique --------------------------------------------------- */
/* Usage : DBG_PRINT(DEBUG_DSP, "valeur=%d\r\n", x);
 * Se comporte exactement comme SafePrintf(...) quand le flag vaut 1,
 * et disparait completement (aucune instruction generee) quand il vaut 0. */
#define DBG_PRINT(flag, ...) do { if (flag) { SafePrintf(__VA_ARGS__); } } while (0)

#endif /* INC_DEBUG_CONFIG_H_ */
