# CBUS User Interface

CANLocking supports the standard CBUS User Interface of a Green LED , Yellow LED and a "FLiM" switch.

  * If the Green LED is lit, CANLocking is in SLiM mode
  * If the Yellow LED is lit, CANLocking is in FLiM mode

## Switching between SLiM and FLiM modes

CANLocking can be switched into FLiM mode by pressing and holding the FLiM switch for greater than 6 seconds, after which the Yellow LED should start flashing and any connected FCU or JMRI should indicate a node is requesting configuration as a FLiM module.  Once registered in the FCU / JMRI the Yellow LED should stop flashing and should be stead ON.

Should the module fail to negotiate with the FCU or JMRI, it will timeout and default to SLiM mode.

You can revert FLiM CANLocking to SLiM by once again pressing and holding the FLiM button until the Yellow LED goes OFF and the Green LED comes ON.

## CBUS CAN Traffic indication

Depending on the SLiM / FLiM state of the module, the other (normally off) LED will indicate activity on the CBUS CAN bus.

<div class="section_buttons">
 
| Previous                      |
|:------------------------------|
| [Node Variables](nodevars.md) |
 
</div>

