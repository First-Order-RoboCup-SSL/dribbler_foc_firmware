## Run
```
make clean
make -j

```
## Debug in Ozone
init to 1 one time 

#### MODE
mode = 0: Open Control Mode
mode = 3: Current Control Mode

#### STARTUP ENCODRER CHECK SEQUENCE
1. set `g_dbg_daxis_lock` to 1 
2. set `g_openloop_vf_offset_dbg` to 0.6, `g_openloop_vq_limit_dbq` limits this offset value
3. set `g_dbg_motor_start` to 1, check that motor is locked into some angle
3. set `g_openloop_omega_e_dbg` to 10 -> 20 -> 50, obverse the encoder changes **continuously** from ozone Timeline, if drop occasioanlly to zero this is a error frame thingy, check for alignment in AS5048A and diametric magnetc


#### RUN CURRENT LOOP
1. always set `g_dbg_motor_start` to `0` before starting anything
2. all openloop dbg variables doesn't matter for this
3. set `mode` to `3`
4. make sure `g_dbq_id_target` and `g_dbq_iq_target` are set to safe values below `0.7` (?) 
 current commands
5. start motor

#### NOTEs
- for torque generating you only need `iq`, set `id` to zero
- for `iq` set to 0.1, sometimes it fails to kick-off automatically, shld not be a problem for bigger

PLOT PHASE CURRENTS! they will be sinusoidal


#### reference safe current draw from PSU
- for current mode control at a `iq` command of 0.5 the current should not exceed ~220mA

## Structure


## Supporting MCU

