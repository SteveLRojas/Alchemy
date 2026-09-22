import serial
import serial.tools.list_ports
import time
from queue import Queue
import tmc5130_regs as regs

baud = 115200
ser = serial.Serial()
power_off = 1

rtmi_responses = []
for i in range(8):
	rtmi_responses.append(Queue(maxsize = 1000000))

def write_reg(address, value):
	address = address | 0x8000
	bytes = address.to_bytes(2, 'big') + value.to_bytes(4, 'big')
	ser.write(bytes)

def read_reg(address):
	bytes = address.to_bytes(2, 'big')
	ser.write(bytes)
	response = ser.read(5)
	return int.from_bytes(response[1:], 'big')

def configure_rtmi(trig_mode, trig_chan, num_chans, continuous_sampling, trigger, num_samples, threshold):
	address = regs.FR_RTMI_THRESHOLD
	bytes = address.to_bytes(2, 'big') + threshold.to_bytes(4, 'big')
	ser.write(bytes)
	address = regs.FR_RTMI_NUM_SAMPLES
	bytes = address.to_bytes(2, 'big') + num_samples.to_bytes(4, 'big')
	ser.write(bytes)
	address = regs.FR_RTMI_CONTROL
	value = trigger & 0x01
	value = (value << 1) | (continuous_sampling & 0x01)
	value = value << 2
	value = (value << 4) | (num_chans & 0x0F)
	value = (value << 4) | (trig_chan & 0x0F)
	value = (value << 4) | (trig_mode & 0x0F)
	bytes = address.to_bytes(2, 'big') + value.to_bytes(4, 'big')
	ser.write(bytes)

def process_rtmi_responses():
	while ser.in_waiting >= 5:
		response = ser.read(5)
		channel_id = int.from_bytes(response[0:1], 'big')
		value = int.from_bytes(response[1:5], 'big')
		if(channel_id > 7):
			print(f"Bad channel ID: 0x{channel_id:02X}, Value: 0x{value:08X}")
		else:
			rtmi_responses[channel_id].put(value)

def flush_rtmi():
	process_rtmi_responses()
	for d in range(8):
		rtmi_responses[d].queue.clear()

def init(unique_id = 0):
	port_candidates = []
	ports = serial.tools.list_ports.comports()
	for port in ports:
		if hasattr(port, 'vid') and port.vid and (port.vid == 0x1A86) and (port.pid == 0xFE0C):
			port_candidates.append(port.device)

	ser.baudrate = baud
	ser.dsrdtr = False
	ser.dtr = False
	ser.timeout = 1.0
	ser.write_timeout = 1.0

	for port in port_candidates:
		try:
			ser.port = port
			ser.open()
			time.sleep(0.5)
			ser.reset_input_buffer()
		except Exception:
			continue
		device_id = read_reg(regs.FR_DEVICE_ID)
		if device_id != 0x30333135:
			ser.close()
			continue
		current_unique_id = read_reg(regs.FR_UNIQUE_ID)
		if unique_id and (unique_id != current_unique_id):
			ser.close()
			continue

		write_reg(regs.FR_RTMI_CONTROL, 0x0000)
		time.sleep(0.5)
		ser.reset_input_buffer()
		return current_unique_id
	return 0

def tmc_stop_platform():
	if ser.is_open:
		if power_off:
			write_reg(regs.R_VMAX, 0)
			write_reg(regs.FR_PIN_SET, 0x01) #Pin Set DRV_EN
		print("Done!")
		ser.close()
