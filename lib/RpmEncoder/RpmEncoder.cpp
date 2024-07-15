
#include "RpmEncoder.h"

// Yes, all the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

RpmEncoder_internal_state_t * RpmEncoder::interruptArgs[];


