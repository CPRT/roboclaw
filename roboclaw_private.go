package roboclaw

import (
	"encoding/binary"
	"time"
)

//How many times to re attempt sending and receiving data
const maxretry = 3

type crcType uint16

/*
 *Variadic function to update the crc
 * @param data ([]uint8) the bytes to update the crc
 */
func (crc *crcType) update(data... uint8) {
	for _, n := range data {
		*crc = *crc ^ (crcType(n) << 8)
		for i := 0; i<8; i++ {
			if (*crc & 0x8000) != 0 {
				*crc = (*crc << 1) ^ 0x1021
			} else {
				*crc <<= 1
			}
		}
	}
}

/*
 * Read the desired number of bytes from the serial bus
 * @param buffer ([]uint8) a byte slice of sufficient size to store the desired number of bytes
 * @return (int, error) the number of bytes returned and any errors
 */
func (r *Roboclaw) read_bytes(buffer []uint8) (int, error) {
	var i int
	//Read the exact number of bytes from the port that will fit into the given slice
	for i = 0; i < len(buffer); {
		if num, err := r.port.Read(buffer[i:]); err != nil {
			return i+num, err
		} else {
			//Increment the index to read from
			i += num
		}
	}
	return i, nil
}

/*
 * Writes a number of bytes to the given roboclaw address
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @param vals ([]uint8) any additional values to transmit
 * @returns (bool) transmission success or failure
 */
func (r *Roboclaw) write_n(address uint8, cmd uint8, vals... uint8) bool {

	var crc crcType = crcType(0)
	vals = append([]uint8{address, cmd}, vals...)
	crc.update(vals...)
	vals = append(vals, uint8(0xFF & (crc >> 8)), uint8(0xFF & crc))
	
	for trys := maxretry; trys > 0 ; trys-- {
		r.port.Flush()
		r.port.Write(vals)

		if _, err := r.read_bytes(buf); err == nil && buf[0] == 0xFF {
			return true
		}
	}
	return false
}

/*
 * Read a variable number of uint32 from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @param vals ([]*uint32) pointers to the ints that are being read from the bus
 * @returns (bool) transmission success or failure
 */
func (r *Roboclaw) read_n(address uint8, cmd uint8, vals... *uint32) bool {
	var (
		buf []uint8
		success bool = true
		crc crcType
		ccrc crcType = crcType(0)
	)

	for trys := maxretry; trys > 0 ; trys-- {
		r.port.Flush()
		crc = crcType(0)

		crc.update(address, cmd)
		r.port.Write([]uint8{address, cmd})

		for _, val := range vals {
			buf = make([]byte, 4)
			if _, err := r.read_bytes(buf); err != nil {
				success = false
				break
			} else {
				crc.update(buf...)
				//Format each array of four bytes into the int pointer
				*val = binary.BigEndian.Uint32(buf)
			}
		}

		if success == true {
			buf = make([]byte, 2)
			if _, err := r.read_bytes(buf); err == nil {
				ccrc = crcType(buf[0]) << 8
				ccrc |= crcType(buf[1])

				if ccrc == crc {
					return true
				}
			}
		}
	}
	return false
}

/*
 * Read 1 byte from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint8, bool) the returned byte and transmission success or failure
 */
func (r *Roboclaw) read1(address uint8, cmd uint8) (uint8, bool){
	n, ok := r.read_count(1, address, cmd)
	return n[0], ok
}

/*
 * Read 1 short from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint16, bool) the returned short and transmission success or failure
 */
func (r *Roboclaw) read2(address uint8, cmd uint8) (uint16, bool){
	n, ok := r.read_count(2, address, cmd)
	return binary.BigEndian.Uint16(n), ok
}

/*
 * Read 1 int from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint32, bool) the returned int and transmission success or failure
 */
func (r *Roboclaw) read4(address uint8, cmd uint8) (uint32, bool){
	n, ok := r.read_count(4, address, cmd)
	return binary.BigEndian.Uint32(n), ok
}

/*
 * Read the specified number of bytes from the roboclaw.
 * Only 1, 2, or 4 bytes may be read. Any other values trigger a panic.
 * @param count (uint8) the number of bytes to read
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns ([]uint8, bool) the returned bytes and transmission success or failure
 */
func (r *Roboclaw) read_count(count uint8, address uint8, cmd uint8) ([]uint8, bool){

	var (
		buf []uint8
		crc crcType
		ccrc crcType = crcType(0)
	)

	//Only allowed values are 1, 2, and 4
	if count != 1 && count != 2 && count != 4 {
		panic("Cannot read count for values other than 1, 2, and 4")
	}

	for trys := maxretry; trys > 0 ; trys-- {
		r.port.Flush()
		crc = crcType(0)

		crc.update(address, cmd)
		r.port.Write([]uint8{address, cmd})

		//Create the appropriate buffer size depending on how 
		// many bytes will be read
		buf = make([]uint8, count+2)
		if _, err := r.read_bytes(buf); err == nil {

			//Update the crc with the data bytes
			crc.update(buf[0:count]...)

			//The final two bytes are the crc from the motor controller
			ccrc = crcType(buf[count]) << 8
			ccrc |= crcType(buf[count+1])

			if ccrc == crc {
				return buf[0:count], true
			}
		}
	}
	return make([]uint8, count), false
}

/*
 * Read four bytes from the roboclaw along with a status flag.
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint32, uint8, bool) the returned bytes formatted into a single int and a status byte and transmission success or failure
 */
func (r *Roboclaw) read4_1(address uint8, cmd uint8) (uint32, uint8, bool){

	var (
		buf []uint8
		crc crcType
		ccrc crcType = crcType(0)
		value uint32 = 0
		status uint8
	)

	for trys := maxretry; trys > 0 ; trys-- {
		r.port.Flush()
		crc = crcType(0)

		crc.update(address, cmd)
		r.port.Write([]uint8{address, cmd})

		buf = make([]uint8, 7)
		if _, err := r.read_bytes(buf); err == nil {

			//Format the first four data bytes into the int
			value = binary.BigEndian.Uint32(buf[0:4])

			//The status byte
			status = buf[4]

			//Update the crc only with the data bytes
			crc.update(buf[0:5]...)

			//The final two bytes are the crc from the motor controller
			ccrc = crcType(buf[5]) << 8
			ccrc |= crcType(buf[6])

			if ccrc == crc {
				return value, status, true
			}
		}
	}
	return 0, 0, false
}
