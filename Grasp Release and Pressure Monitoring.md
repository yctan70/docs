## Grasp/Release and Pressure Monitoring Walkthrough

This document describes the changes made to `ls_hand_pro.c`  and `ls_hand.c`.

### `ls_hand_pro.c`  (Using LHandProLib)

-   Implemented a periodic grasp/release cycle every 3 seconds.
-   Added pressure monitoring for 5 fingertips.
-   Build:  `make ls_hand_pro`
-   Run:  `sudo ./ls_hand_pro`


### `ls_hand.c`  (No Library)

-   Ported the same logic to use direct *EtherCAT* commands.
-   Handles raw *PDO* data for axis control and sensor feedback.
-   **Note**: Uses 0-based indexing for axes (0-5), unlike the library's 1-based indexing.
-   Build:  `make ls_hand`
-   Run:  `sudo ./ls_hand`

### Expected Behavior (Both Versions)

The hand will cycle between:

1.  **Grasp**: All fingers move to position 10000.
2.  **Release**: All fingers move to position 0.

The terminal will confirm operation:
```
Cycle 1234 | OP: YES | ... | State: CLOSING/HOLDING
...
Thumb :   0.50 N | Index :   1.20 N | ...
```
