package roboclaw

import (
	"encoding/binary"
	"fmt"
	"time"
)

type crcType uint16

/*
 * Variadic function to update the crc16 value
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
		// Technically, the fact that err != nil when zero bytes are returned
		// is an implementation detail of the file descriptor returned by serial.OpenPort.
		// I think it is better not to rely on it, so this function also
		// returns when zero bytes are returned.
		// Update: It turns out that on windows 10 zero is returned when a timeout occurs,
		// but no error (i.e. nil). By contrast, on linux zero and an error are returned if no bytes
		// are read.
		if num, err := r.port.Read(buffer[i:]); err != nil {
			return i+num, err
		} else if 0 == num {
			return i+num, fmt.Errorf("Read timeout: Zero Bytes Received")
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
 * @returns (error) transmission success or failure
 */
func (r *Roboclaw) write_n(address uint8, cmd uint8, vals... uint8) error {

	var (
		crc crcType = crcType(0)
	    buf []byte
		err error
	)
	vals = append([]uint8{address, cmd}, vals...)
	crc.update(vals...)
	vals = append(vals, uint8(0xFF & (crc >> 8)), uint8(0xFF & crc))
	
	for trys := r.retries; trys > 0 ; trys-- {
		// Empty error (this should never be returned to user, but is employed in case of error in control flow logic)
		// That way the only way nil is returned is if there is success
		err = fmt.Errorf("Assert: This error should be replaced by other errors or nil")
		if err = r.port.Flush(); nil != err {
			continue
		} else if _, err = r.port.Write(vals); nil != err {
			continue
		}

		buf = make([]byte,1)
		// If the writeSleep flag is true, then
		// instead of waiting for a reply simply sleep for the expected
		// timeout (10 milliseconds)
		// This can resolve issues if the rx line is broken but
		// the tx line is still operational, as the extra delay from
		// the wait will no longer be a barrier to efficient communication
		// with the roboclaw
		if r.writeSleep {
			time.Sleep(time.Millisecond*10)
			return nil
		} else if _, err = r.read_bytes(buf); err != nil {
			continue
		} else if buf[0] == 0xFF {
			return nil
		} else {
			err = fmt.Errorf("Incorrect Response Byte")
		}
	}
	return err
}

/*
 * Read a variable number of uint32 from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @param vals ([]*uint32) pointers to the ints that are being read from the bus
 * @returns (error) transmission success or failure
 */
func (r *Roboclaw) read_n(address uint8, cmd uint8, vals... *uint32) error {
	var (
		buf []uint8
		crc crcType
		ccrc crcType
		err error
	)
	loop:
	for trys := r.retries; trys > 0 ; trys-- {
		// Empty error (this should never be returned to user, but is employed in case of error in control flow logic)
		// That way the only way nil is returned is if there is success
		err = fmt.Errorf("Assert: This error should be replaced by other errors or nil")
		if err = r.port.Flush(); nil != err {
			continue
		}

		crc = crcType(0)

		crc.update(address, cmd)
		if _, err = r.port.Write([]uint8{address, cmd}); nil != err {
			continue
		}

		for _, val := range vals {
			buf = make([]byte, 4)
			if _, err = r.read_bytes(buf); err != nil {
				continue loop
			} else {
				crc.update(buf...)
				//Format each array of four bytes into the int pointer
				*val = binary.BigEndian.Uint32(buf)
			}
		}

		buf = make([]byte, 2)
		if _, err = r.read_bytes(buf); err == nil {
			ccrc = crcType(buf[0]) << 8
			ccrc |= crcType(buf[1])

			if ccrc == crc {
				return nil
			} else {
				err = fmt.Errorf("Mismatched checksum values")
			}
		}
	}
	return err
}

/*
 * Read 1 byte from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint8, error) the returned byte and any error
 */
func (r *Roboclaw) read1(address uint8, cmd uint8) (uint8, error){
	n, err := r.read_count(1, address, cmd)
	return n[0], err
}

/*
 * Read 1 16 bit short from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint16, error) the returned short and any error
 */
func (r *Roboclaw) read2(address uint8, cmd uint8) (uint16, error){
	n, err := r.read_count(2, address, cmd)
	return binary.BigEndian.Uint16(n), err
}

/*
 * Read 1 32 bit int from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint32, error) the returned int and any error
 */
func (r *Roboclaw) read4(address uint8, cmd uint8) (uint32, error){
	n, err := r.read_count(4, address, cmd)
	return binary.BigEndian.Uint32(n), err
}

/*
 * Read 2 32 bit ints from the roboclaw
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint32, uint32, error) the returned ints and any error
 */
func (r *Roboclaw) read4_4(address uint8, cmd uint8) (uint32, uint32, error){
	n, err := r.read_count(8, address, cmd)
	return binary.BigEndian.Uint32(n[0:4]), binary.BigEndian.Uint32(n[4:8]), err
}

/*
 * Read four bytes from the roboclaw along with a status flag.
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns (uint32, uint8, error) the returned bytes formatted into a single int and a status byte and any error
 */
func (r *Roboclaw) read4_1(address uint8, cmd uint8) (uint32, uint8, error){
	n, err := r.read_count(5, address, cmd)
	return binary.BigEndian.Uint32(n[0:4]), n[4], err
}

/*
 * Read the specified number of bytes from the roboclaw.
 * Only 1, 2, or 4 bytes may be read. Any other values trigger a panic.
 * @param count (uint8) the number of bytes to read
 * @param address (uint8) the roboclaw address
 * @param cmd (uint8) the command number
 * @returns ([]uint8, error) the returned bytes and any error
 */
func (r *Roboclaw) read_count(count uint8, address uint8, cmd uint8) ([]uint8, error){

	var (
		buf []uint8
		crc crcType
		ccrc crcType
		err error
	)

	//Only allowed values are 1, 2, and 4
	if count != 1 && count != 2 && count != 4 && count != 5 && count != 8 {
		panic("Cannot read count for values other than 1, 2, 4, 5, and 8")
	}

	for trys := r.retries; trys > 0 ; trys-- {
		// Empty error (this should never be returned to user, but is employed in case of error in control flow logic)
		// That way the only way nil is returned is if there is success
		err = fmt.Errorf("Assert: This error should be replaced by other errors or nil")
		if err = r.port.Flush(); err != nil {
			continue
		}
		crc = crcType(0)

		crc.update(address, cmd)
		if _, err = r.port.Write([]uint8{address, cmd}); err != nil {
			continue
		}

		//Create the appropriate buffer size depending on how
		// many bytes will be read
		buf = make([]uint8, count+2)
		if _, err = r.read_bytes(buf); err == nil {

			//Update the crc with the data bytes
			crc.update(buf[0:count]...)

			//The final two bytes are the crc from the motor controller
			ccrc = crcType(buf[count]) << 8
			ccrc |= crcType(buf[count+1])

			if ccrc == crc {
				return buf[0:count], nil
			} else {
				err = fmt.Errorf("Mismatched checksum values")
			}
		}
	}
	return make([]uint8, count), err
}
