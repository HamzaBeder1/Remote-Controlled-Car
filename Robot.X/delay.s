;Description: API for sending data to the iLED. Used to show the selected color and if the follower's guess was
;correct or not
.include "xc.inc"

.text                      


.global  _wait_16000cycles
    
    
_wait_16000cycles:
    repeat #15993
    nop
    return



