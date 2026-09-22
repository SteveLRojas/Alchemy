import time
import tmc5130_regs as regs
import alchemy_tmc5130_platform as plat

def main():
    plat.init()

    #do the thing here
    toff = 3
    hstrt = 4
    hend = 1
    tbl = 2
    chm = 0
    val = (tbl << 15) | (chm << 14) | (hend << 7) | (hstrt << 4) | toff
    plat.write_reg(regs.R_CHOPCONF, val)

    ihold = 10
    irun = 15
    iholddelay = 6
    val = (iholddelay << 16) | (irun << 8) | ihold
    plat.write_reg(regs.R_IHOLD_IRUN, val)

    plat.write_reg(regs.R_TPOWERDOWN, 10)
    plat.write_reg(regs.R_RAMPMODE, 0)   #position mode
    plat.write_reg(regs.R_VMAX, 0)

    plat.write_reg(regs.R_A1, 4000)
    plat.write_reg(regs.R_AMAX, 4000)
    plat.write_reg(regs.R_DMAX, 4000)
    plat.write_reg(regs.R_D1, 4000)

    plat.write_reg(regs.R_VMAX, 400000)
    plat.write_reg(regs.R_VSTART, 100)
    plat.write_reg(regs.R_V1, 200)
    plat.write_reg(regs.R_VSTOP, 100)

    plat.write_reg(regs.FR_PIN_CLEAR, 0x01) #Pin Clear DRV_EN
    time.sleep(0.1)

    steps_per_rotation = 200
    num_rotations = 50
    x_target = 0xFFFFFFFF & (num_rotations * 256 * steps_per_rotation + plat.read_reg(regs.R_XACTUAL))
    plat.write_reg(regs.R_XTARGET, x_target)

    ramp_stat = 0
    while (ramp_stat & 0x200) == 0:
       ramp_stat = plat.read_reg(regs.R_RAMP_STAT)
    print(f"ramp_stat: {plat.read_reg(regs.R_RAMP_STAT):08X}")

    time.sleep(2.0)
    plat.tmc_stop_platform()


if __name__ == "__main__":
    main()
