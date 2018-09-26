package roboclaw

import (
	"github.com/tarm/serial"
	"time"
)

// A structure for describing the roboclaw interface
type Roboclaw struct {
	port *serial.Port
}

/*
 * Initialize the roboclaw with the desired serial port
 * @param name (string) the name of the serial port
 * @param baud (int) the baud rate for the serial port
 * @returns (*Roboclaw, error) the reference to the roboclaw and any errors that occur when opening the file
 */
func Init(name string, baud int) (*Roboclaw, error) {
	c := &serial.Config{Name: name, Baud: baud, ReadTimeout: time.Millisecond*10}

	if port, err := serial.OpenPort(c); err == nil {
		return &Roboclaw{port: port}, err
	} else {
		return nil, err
	}
}

/*
 * Close the roboclaw's serial port
 */
func (r *Roboclaw) Close() {
	r.port.Close()
}

func (r *Roboclaw) ForwardM1(address uint8, speed uint8) bool {
	return r.write_n(address, M1FORWARD, speed)
}

func (r *Roboclaw) BackwardM1(address uint8, speed uint8) bool {
	return r.write_n(address, M1BACKWARD, speed)
}

func (r *Roboclaw) ForwardM2(address uint8, speed uint8) bool {
	return r.write_n(address, M2FORWARD, speed)
}

func (r *Roboclaw) BackwardM2(address uint8, speed uint8) bool {
	return r.write_n(address, M2BACKWARD, speed)
}

func (r *Roboclaw) SetMinVoltageMainBattery(address uint8, voltage uint8) bool {
	return r.write_n(address, SETMINMB, voltage)
}

func (r *Roboclaw) SetMaxVoltageMainBattery(address uint8, voltage uint8) bool {
	return r.write_n(address, SETMAXMB, voltage)
}

func (r *Roboclaw) ForwardBackwardM1(address uint8, speed uint8) bool {
	return r.write_n(address, M17BIT, speed)
}

func (r *Roboclaw) ForwardBackwardM2(address uint8, speed uint8) bool {
	return r.write_n(address, M27BIT, speed)
}

func (r *Roboclaw) ForwardMixed(address uint8, speed uint8) bool {
	return r.write_n(address, MIXEDFORWARD, speed)
}

func (r *Roboclaw) BackwardMixed(address uint8, speed uint8) bool {
	return r.write_n(address, MIXEDBACKWARD, speed)
}

func (r *Roboclaw) TurnRightMixed(address uint8, speed uint8) bool {
	return r.write_n(address, MIXEDRIGHT, speed)
}

func (r *Roboclaw) TurnLeftMixed(address uint8, speed uint8) bool {
	return r.write_n(address, MIXEDLEFT, speed)
}

func (r *Roboclaw) ForwardBackwardMixed(address uint8, speed uint8) bool {
	return r.write_n(address, MIXEDFB, speed)
}

func (r *Roboclaw) LeftRightMixed(address uint8, speed uint8) bool {
	return r.write_n(address, MIXEDLR, speed)
}

func (r *Roboclaw) ReadEncM1(address uint8) (uint32, uint8, bool) {
	return r.read4_1(address, GETM1ENC)
}

func (r *Roboclaw) ReadEncM2(address uint8) (uint32, uint8, bool) {
	return r.read4_1(address, GETM2ENC)
}

func (r *Roboclaw) ReadSpeedM1(address uint8) (uint32, uint8, bool) {
	return r.read4_1(address, GETM1SPEED)
}

func (r *Roboclaw) ReadSpeedM2(address uint8) (uint32, uint8, bool) {
	return r.read4_1(address, GETM2SPEED)
}

func (r *Roboclaw) ResetEncoders(address uint8) bool {
	return r.write_n(address, RESETENC)
}

func (r *Roboclaw) ReadVersion(address uint8) (string, bool) {
	var (
		version []uint8 = make([]uint8, 0)
		crc crcType
		ccrc uint16
		buf []uint8 = make([]uint8, 1)
	)

	for trys := maxretry; trys > 0 ; trys-- {
		r.port.Flush()
		crc = crcType(0)

		crc.update(address, GETVERSION)
		r.port.Write([]uint8{address, GETVERSION})

		for i := 0; i < 48; i++ {
			if _, err := r.read_bytes(buf); err == nil {

				//Use append to ensure that version is no longer than it
				//needs to be
				version = append(version, buf[0])
				crc.update(buf[0])

				if buf[0] == 0 {

					buf = make([]uint8, 2)
					if _, err = r.read_bytes(buf); err == nil {
						ccrc = uint16(buf[0]) << 8
						ccrc |= uint16(buf[1])

						if ccrc == uint16(crc) {
							return string(version), true
						}
					}
				}
			} else {
				break
			}
		}
	}
	return "", false
}

func (r *Roboclaw) SetEncM1(address uint8, val int32) bool {
	return r.write_n(address, SETM1ENCCOUNT, setDWORDval(uint32(val))...)
}

func (r *Roboclaw) SetEncM2(address uint8, val int32) bool {
	return r.write_n(address, SETM2ENCCOUNT, setDWORDval(uint32(val))...)
}

func (r *Roboclaw) ReadMainBatteryVoltage(address uint8) (uint16, bool) {
	return r.read2(address, GETMBATT)
}

func (r *Roboclaw) ReadLogicBatteryVoltage(address uint8) (uint16, bool) {
	return r.read2(address, GETLBATT)
}

func (r *Roboclaw) SetMinVoltageLogicBattery(address uint8, voltage uint8) bool {
	return r.write_n(address, SETMINLB, voltage)
}

func (r *Roboclaw) SetMaxVoltageLogicBattery(address uint8, voltage uint8) bool {
	return r.write_n(address, SETMAXLB, voltage)
}

func (r *Roboclaw) SetM1VelocityPID(address uint8, kp_fp float32, ki_fp float32, kd_fp float32, qpps uint32) bool {
	array := append(setDWORDval(uint32(kp_fp*65536)), setDWORDval(uint32(ki_fp*65536))...)
	array = append(array, setDWORDval(uint32(kd_fp*65536))...)
	array = append(array, setDWORDval(qpps)...)
	return r.write_n(address, SETM1PID, array...)
}

func (r *Roboclaw) SetM2VelocityPID(address uint8, kp_fp float32, ki_fp float32, kd_fp float32, qpps uint32) bool {
	array := append(setDWORDval(uint32(kp_fp*65536)), setDWORDval(uint32(ki_fp*65536))...)
	array = append(array, setDWORDval(uint32(kd_fp*65536))...)
	array = append(array, setDWORDval(qpps)...)
	return r.write_n(address, SETM2PID, array...)
}

func (r *Roboclaw) ReadISpeedM1(address uint8) (uint32, uint8, bool) {
	return r.read4_1(address, GETM1ISPEED)
}

func (r *Roboclaw) ReadISpeedM2(address uint8) (uint32, uint8, bool) {
	return r.read4_1(address, GETM2ISPEED)
}

func (r *Roboclaw) DutyM1(address uint8, duty uint16) bool {
	return r.write_n(address, M1DUTY, setWORDval(duty)...)
}

func (r *Roboclaw) DutyM2(address uint8, duty uint16) bool {
	return r.write_n(address, M2DUTY, setWORDval(duty)...)
}

func (r *Roboclaw) DutyM1M2(address uint8, duty1 uint16, duty2 uint16) bool {
	return r.write_n(address, MIXEDDUTY, append(setWORDval(duty1), setWORDval(duty2)...)...)
}

func (r *Roboclaw) SpeedM1(address uint8, speed uint32) bool {
	return r.write_n(address, M1SPEED, setDWORDval(speed)...)
}

func (r *Roboclaw) SpeedM2(address uint8, speed uint32) bool {
	return r.write_n(address, M2SPEED, setDWORDval(speed)...)
}

func (r *Roboclaw) SpeedM1M2(address uint8, speed1 uint32, speed2 uint32) bool {
	return r.write_n(address, MIXEDSPEED, append(setDWORDval(speed1), setDWORDval(speed2)...)...)
}

func (r *Roboclaw) SpeedAccelM1(address uint8, accel uint32, speed uint32) bool {
	return r.write_n(address, M1SPEEDACCEL, append(setDWORDval(accel), setDWORDval(speed)...)...)
}

func (r *Roboclaw) SpeedAccelM2(address uint8, accel uint32, speed uint32) bool {
	return r.write_n(address, M2SPEEDACCEL, append(setDWORDval(accel), setDWORDval(speed)...)...)
}

func (r *Roboclaw) SpeedAccelM1M2(address uint8, accel uint32, speed1 uint32, speed2 uint32) bool {
	array := append(setDWORDval(accel), setDWORDval(speed1)...)
	return r.write_n(address, MIXEDSPEEDACCEL, append(array, setDWORDval(speed2)...)...)
}

func (r *Roboclaw) SpeedDistanceM1(address uint8, speed uint32, distance uint32, flag uint8) bool {
	array := append(setDWORDval(speed), setDWORDval(distance)...)
	return r.write_n(address, M1SPEEDDIST, append(array, flag)...)
}

func (r *Roboclaw) SpeedDistanceM2(address uint8, speed uint32, distance uint32, flag uint8) bool {
	array := append(setDWORDval(speed), setDWORDval(distance)...)
	return r.write_n(address, M2SPEEDDIST, append(array, flag)...)
}

func (r *Roboclaw) SpeedDistanceM1M2(address uint8, speed1 uint32, distance1 uint32, speed2 uint32, distance2 uint32, flag uint8) bool {
	array := append(setDWORDval(speed1), setDWORDval(distance1)...)
	array = append(array, setDWORDval(speed2)...)
	array = append(array, setDWORDval(distance2)...)
	return r.write_n(address, MIXEDSPEEDDIST, append(array, flag)...)
}

func (r *Roboclaw) SpeedAccelDistanceM1(address uint8, accel uint32, speed uint32, distance uint32, flag uint8) bool {
	array := append(setDWORDval(accel), setDWORDval(speed)...)
	array = append(array, setDWORDval(distance)...)
	return r.write_n(address, M1SPEEDACCELDIST, append(array, flag)...)
}

func (r *Roboclaw) SpeedAccelDistanceM2(address uint8, accel uint32, speed uint32, distance uint32, flag uint8) bool {
	array := append(setDWORDval(accel), setDWORDval(speed)...)
	array = append(array, setDWORDval(distance)...)
	return r.write_n(address, M2SPEEDACCELDIST, append(array, flag)...)
}

func (r *Roboclaw) SpeedAccelDistanceM1M2(address uint8, accel uint32, speed1 uint32, distance1 uint32, speed2 uint32, distance2 uint32, flag uint8) bool {
	array := append(setDWORDval(accel), setDWORDval(speed1)...)
	array = append(array, setDWORDval(distance1)...)
	array = append(array, setDWORDval(speed2)...)
	array = append(array, setDWORDval(distance2)...)
	return r.write_n(address, MIXEDSPEEDACCELDIST, append(array, flag)...)
}

func (r *Roboclaw) ReadBuffers(address uint8) (uint8, uint8, bool) {
	value, valid := r.read2(address, GETBUFFERS)
	return uint8(value >> 8), uint8(value), valid
}

func (r *Roboclaw) ReadPWMs(address uint8) (int16, int16, bool) {
	value, valid := r.read4(address, GETPWMS)
	return int16(value >> 16), int16(value), valid
}

func (r *Roboclaw) ReadCurrents(address uint8) (int16, int16, bool) {
	value, valid := r.read4(address, GETCURRENTS)
	return int16(value >> 16), int16(value), valid
}

func (r *Roboclaw) SpeedAccelM1M2_2(address uint8, accel1 uint32, speed1 uint32, accel2 uint32, speed2 uint32) bool {
	array := append(setDWORDval(accel1), setDWORDval(speed1)...)
	array = append(array, setDWORDval(accel2)...)
	array = append(array, setDWORDval(speed2)...)
	return r.write_n(address, MIXEDSPEED2ACCEL, array...)
}

func (r *Roboclaw) SpeedAccelDistanceM1M2_2(address uint8, accel1 uint32, speed1 uint32, distance1 uint32, accel2 uint32, speed2 uint32, distance2 uint32, flag uint8) bool {
	array := append(setDWORDval(accel1), setDWORDval(speed1)...)
	array = append(array, setDWORDval(distance1)...)
	array = append(array, setDWORDval(accel2)...)
	array = append(array, setDWORDval(speed2)...)
	array = append(array, setDWORDval(distance2)...)
	return r.write_n(address, MIXEDSPEED2ACCELDIST, append(array, flag)...)
}

func (r *Roboclaw) DutyAccelM1(address uint8, duty uint16, accel uint32) bool {
	return r.write_n(address, M1DUTYACCEL, append(setWORDval(duty), setDWORDval(accel)...)...)
}

func (r *Roboclaw) DutyAccelM2(address uint8, duty uint16, accel uint32) bool {
	return r.write_n(address, M2DUTYACCEL, append(setWORDval(duty), setDWORDval(accel)...)...)
}

func (r *Roboclaw) DutyAccelM1M2(address uint8, duty1 uint16, accel1 uint32, duty2 uint16, accel2 uint32) bool {
	array := append(setWORDval(duty1), setDWORDval(accel1)...)
	array = append(array, setWORDval(duty2)...)
	array = append(array, setDWORDval(accel2)...)
	return r.write_n(address, MIXEDDUTYACCEL, array...)
}

func (r *Roboclaw) ReadM1VelocityPID(address uint8) (float32, float32, float32, uint32, bool) {
	var Kp, Ki, Kd, qpps uint32
	valid := r.read_n(address, READM1PID, &Kp, &Ki, &Kd, &qpps)
	return float32(Kp/65536), float32(Ki/65536), float32(Kd/65536), qpps, valid
}

func (r *Roboclaw) ReadM2VelocityPID(address uint8) (float32, float32, float32, uint32, bool) {
	var Kp, Ki, Kd, qpps uint32
	valid := r.read_n(address, READM2PID, &Kp, &Ki, &Kd, &qpps)
	return float32(Kp/65536), float32(Ki/65536), float32(Kd/65536), qpps, valid
}

func (r *Roboclaw) SetMainVoltages(address uint8, min uint16, max uint16) bool {
	return r.write_n(address, SETMAINVOLTAGES, append(setWORDval(min), setWORDval(max)...)...)
}

func (r *Roboclaw) SetLogicVoltages(address uint8, min uint16, max uint16) bool {
	return r.write_n(address, SETLOGICVOLTAGES, append(setWORDval(min), setWORDval(max)...)...)
}

func (r *Roboclaw) ReadMinMaxMainVoltages(address uint8) (uint16, uint16, bool) {
	value, valid := r.read4(address, GETMINMAXMAINVOLTAGES)
	return uint16(value >> 16), uint16(value), valid
}

func (r *Roboclaw) ReadMinMaxLogicVoltages(address uint8) (uint16, uint16, bool) {
	value, valid := r.read4(address, GETMINMAXLOGICVOLTAGES)
	return uint16(value >> 16), uint16(value), valid
}

func (r *Roboclaw) SetM1PositionPID(address uint8, kp_fp float32, ki_fp float32, kd_fp float32, kiMax uint32, deadzone uint32, min uint32, max uint32) bool {
	array := append(setDWORDval(uint32(kp_fp*1024)), setDWORDval(uint32(ki_fp*1024))...)
	array = append(array, setDWORDval(uint32(kd_fp*1024))...)
	array = append(array, setDWORDval(kiMax)...)
	array = append(array, setDWORDval(deadzone)...)
	array = append(array, setDWORDval(min)...)
	array = append(array, setDWORDval(max)...)
	return r.write_n(address, SETM1POSPID, array...)
}

func (r *Roboclaw) SetM2PositionPID(address uint8, kp_fp float32, ki_fp float32, kd_fp float32, kiMax uint32, deadzone uint32, min uint32, max uint32) bool {
	array := append(setDWORDval(uint32(kp_fp*1024)), setDWORDval(uint32(ki_fp*1024))...)
	array = append(array, setDWORDval(uint32(kd_fp*1024))...)
	array = append(array, setDWORDval(kiMax)...)
	array = append(array, setDWORDval(deadzone)...)
	array = append(array, setDWORDval(min)...)
	array = append(array, setDWORDval(max)...)
	return r.write_n(address, SETM2POSPID, array...)
}

func (r *Roboclaw) ReadM1PositionPID(address uint8) (float32, float32, float32, uint32, uint32, uint32, uint32, bool) {
	var Kp, Ki, Kd, KiMax, DeadZone, Min, Max uint32
	valid := r.read_n(address, READM1POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max)
	return float32(Kp/1024), float32(Ki/1024), float32(Kd/1024), KiMax, DeadZone, Min, Max, valid
}

func (r *Roboclaw) ReadM2PositionPID(address uint8) (float32, float32, float32, uint32, uint32, uint32, uint32, bool) {
	var Kp, Ki, Kd, KiMax, DeadZone, Min, Max uint32
	valid := r.read_n(address, READM2POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max)
	return float32(Kp/1024), float32(Ki/1024), float32(Kd/1024), KiMax, DeadZone, Min, Max, valid
}

func (r *Roboclaw) SpeedAccelDeccelPositionM1(address uint8, accel uint32, speed uint32, deccel uint32, position uint32, flag uint8) bool {
	array := append(setDWORDval(accel), setDWORDval(speed)...)
	array = append(array, setDWORDval(deccel)...)
	array = append(array, setDWORDval(position)...)
	return r.write_n(address, M1SPEEDACCELDECCELPOS, append(array, flag)...)
}

func (r *Roboclaw) SpeedAccelDeccelPositionM2(address uint8, accel uint32, speed uint32, deccel uint32, position uint32, flag uint8) bool {
	array := append(setDWORDval(accel), setDWORDval(speed)...)
	array = append(array, setDWORDval(deccel)...)
	array = append(array, setDWORDval(position)...)
	return r.write_n(address, M2SPEEDACCELDECCELPOS, append(array, flag)...)
}

func (r *Roboclaw) SpeedAccelDeccelPositionM1M2(address uint8, accel1 uint32, speed1 uint32, deccel1 uint32, position1 uint32, accel2 uint32, speed2 uint32, deccel2 uint32, position2 uint32, flag uint8) bool {
	array := append(setDWORDval(accel1), setDWORDval(speed1)...)
	array = append(array, setDWORDval(deccel1)...)
	array = append(array, setDWORDval(position1)...)
	array = append(array, setDWORDval(accel2)...)
	array = append(array, setDWORDval(speed2)...)
	array = append(array, setDWORDval(deccel2)...)
	array = append(array, setDWORDval(position2)...)
	return r.write_n(address, MIXEDSPEEDACCELDECCELPOS, append(array, flag)...)
}

func (r *Roboclaw) SetM1DefaultAccel(address uint8, accel uint32) bool {
	return r.write_n(address, SETM1DEFAULTACCEL, setDWORDval(accel)...)
}

func (r *Roboclaw) SetM2DefaultAccel(address uint8, accel uint32) bool {
	return r.write_n(address, SETM2DEFAULTACCEL, setDWORDval(accel)...)
}

func (r *Roboclaw) SetPinFunctions(address uint8, s3mode uint8, s4mode uint8, s5mode uint8) bool {
	return r.write_n(address, SETPINFUNCTIONS, s3mode, s4mode, s5mode)
}

func (r *Roboclaw) GetPinFunctions(address uint8) (uint8, uint8, uint8, bool) {
	var (
		crc crcType
		ccrc uint16
		buf []uint8
	)

	for trys := maxretry; trys > 0 ; trys-- {
		r.port.Flush()
		crc = crcType(0)

		crc.update(address, GETPINFUNCTIONS)
		r.port.Write([]uint8{address, GETPINFUNCTIONS})

		buf = make([]uint8, 5)
		if _, err := r.read_bytes(buf); err == nil {
			crc.update(buf[0:3]...)

			ccrc = uint16(buf[3]) << 8
			ccrc |= uint16(buf[4])

			if ccrc == uint16(crc) {
				return buf[0], buf[1], buf[2], true
			}
		}
	}
	return 0,0,0,false
}

func (r *Roboclaw) SetDeadBand(address uint8, Min uint8, Max uint8) bool {
	return r.write_n(address, SETDEADBAND, Min, Max)
}

func (r *Roboclaw) GetDeadBand(address uint8) (uint8, uint8, bool) {
	value, valid := r.read2(address, GETDEADBAND)
	return uint8(value >> 8), uint8(value), valid
}

func (r *Roboclaw) ReadEncoders(address uint8) (uint32, uint32, bool) {
	var enc1, enc2 uint32
	valid := r.read_n(address, GETENCODERS, &enc1, &enc2)
	return enc1, enc2, valid
}

func (r *Roboclaw) ReadISpeeds(address uint8) (uint32, uint32, bool) {
	var ispeed1, ispeed2 uint32
	valid := r.read_n(address, GETISPEEDS, &ispeed1, &ispeed2)
	return ispeed1, ispeed2, valid
}

func (r *Roboclaw) RestoreDefaults(address uint8) bool {
	return r.write_n(address, RESTOREDEFAULTS)
}

func (r *Roboclaw) ReadTemp(address uint8) (uint16, bool) {
	return r.read2(address, GETTEMP)
}

func (r *Roboclaw) ReadTemp2(address uint8) (uint16, bool) {
	return r.read2(address, GETTEMP2)
}

func (r *Roboclaw) ReadError(address uint8) (uint16, bool) {
	return r.read2(address, GETERROR)
}

func (r *Roboclaw) ReadEncoderModes(address uint8) (uint8, uint8, bool) {
	value, valid := r.read2(address, GETENCODERMODE)
	return uint8(value >> 8), uint8(value), valid
}

func (r *Roboclaw) SetM1EncoderMode(address uint8, mode uint8) bool {
	return r.write_n(address, SETM1ENCODERMODE, mode)
}

func (r *Roboclaw) SetM2EncoderMode(address uint8, mode uint8) bool {
	return r.write_n(address, SETM2ENCODERMODE, mode)
}

func (r *Roboclaw) WriteNVM(address uint8) bool {
	return r.write_n(address, WRITENVM, setDWORDval(0xE22EAB7A)...)
}

func (r *Roboclaw) ReadNVM(address uint8) bool {
	return r.write_n(address, READNVM)
}

func (r *Roboclaw) SetConfig(address uint8, config uint16) bool {
	return r.write_n(address, SETCONFIG, setWORDval(config)...)
}

func (r *Roboclaw) GetConfig(address uint8) (uint16, bool) {
	return r.read2(address, GETCONFIG)
}

func (r *Roboclaw) SetM1MaxCurrent(address uint8, max uint32) bool {
	return r.write_n(address, SETM1MAXCURRENT, append(setDWORDval(max), setDWORDval(0)...)...)
}

func (r *Roboclaw) SetM2MaxCurrent(address uint8, max uint32) bool {
	return r.write_n(address, SETM2MAXCURRENT, append(setDWORDval(max), setDWORDval(0)...)...)
}

func (r *Roboclaw) ReadM1MaxCurrent(address uint8) (uint32, bool) {
	var tmax, dummy uint32
	valid := r.read_n(address, GETM1MAXCURRENT, &tmax, &dummy)
	return tmax, valid
}

func (r *Roboclaw) ReadM2MaxCurrent(address uint8) (uint32, bool) {
	var tmax, dummy uint32
	valid := r.read_n(address, GETM2MAXCURRENT, &tmax, &dummy)
	return tmax, valid
}

func (r *Roboclaw) SetPWMMode(address uint8, mode uint8) bool {
	return r.write_n(address, SETPWMMODE, mode)
}

func (r *Roboclaw) GetPWMMode(address uint8) (uint8, bool) {
	value, valid := r.read1(address, GETPWMMODE)
	return value, valid
}
