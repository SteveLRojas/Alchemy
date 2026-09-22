# General Configuration Registers (0x00–0x0F)
R_GCONF        = 0x00
R_GSTAT        = 0x01
R_IFCNT        = 0x02
R_SLAVECONF    = 0x03
R_IOIN         = 0x04
R_OUTPUT       = 0x04
R_X_COMPARE    = 0x05

# Velocity Dependent Driver Feature Control (0x10–0x1F)
R_IHOLD_IRUN   = 0x10
R_TPOWERDOWN   = 0x11
R_TSTEP        = 0x12
R_TPWMTHRS     = 0x13
R_TCOOLTHRS    = 0x14
R_THIGH        = 0x15

# Ramp Generator Motion Control Registers (0x20–0x2D)
R_RAMPMODE     = 0x20
R_XACTUAL      = 0x21
R_VACTUAL      = 0x22
R_VSTART       = 0x23
R_A1           = 0x24
R_V1           = 0x25
R_AMAX         = 0x26
R_VMAX         = 0x27
R_DMAX         = 0x28
R_D1           = 0x2A
R_VSTOP        = 0x2B
R_TZEROWAIT    = 0x2C
R_XTARGET      = 0x2D

# Ramp Generator Driver Feature Control (0x30–0x36)
R_VDCMIN       = 0x33
R_SW_MODE      = 0x34
R_RAMP_STAT    = 0x35
R_XLATCH       = 0x36

# Encoder Registers (0x38–0x3C)
R_ENCMODE      = 0x38
R_X_ENC        = 0x39
R_ENC_CONST    = 0x3A
R_ENC_STATUS   = 0x3B
R_ENC_LATCH    = 0x3C

# Microstepping Control Registers (0x60–0x6B)
R_MSLUT0       = 0x60
R_MSLUT1       = 0x61
R_MSLUT2       = 0x62
R_MSLUT3       = 0x63
R_MSLUT4       = 0x64
R_MSLUT5       = 0x65
R_MSLUT6       = 0x66
R_MSLUT7       = 0x67
R_MSLUTSEL     = 0x68
R_MSLUTSTART   = 0x69
R_MSCNT        = 0x6A
R_MSCURACT     = 0x6B

# Driver Registers (0x6C–0x7F)
R_CHOPCONF     = 0x6C
R_COOLCONF     = 0x6D
R_DCCTRL       = 0x6E
R_DRV_STATUS   = 0x6F

R_PWMCONF      = 0x70
R_PWM_SCALE    = 0x71
R_ENCM_CTRL    = 0x72
R_LOST_STEPS   = 0x73

FR_DEVICE_ID = 0x7FFF
FR_UNIQUE_ID = 0x7FFE
FR_PIN_DATA = 0x7FFD
FR_PIN_SET = 0x7FFC
FR_PIN_CLEAR = 0x7FFB
FR_RTMI_CONTROL = 0x7FFA
FR_RTMI_NUM_SAMPLES = 0x7FF9
FR_RTMI_THRESHOLD = 0x7FF8
FR_RTMI_CHANNEL_0 = 0x7FF7
FR_RTMI_CHANNEL_1 = 0x7FF6
FR_RTMI_CHANNEL_2 = 0x7FF5
FR_RTMI_CHANNEL_3 = 0x7FF4
FR_RTMI_CHANNEL_4 = 0x7FF3
FR_RTMI_CHANNEL_5 = 0x7FF2
FR_RTMI_CHANNEL_6 = 0x7FF1
FR_RTMI_CHANNEL_7 = 0x7FF0
FR_RTMI_PERIOD = 0x7FEF
