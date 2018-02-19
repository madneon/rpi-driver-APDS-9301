import smbus, time

REG_CONTROL      = 0x80
REG_TIMING       = 0x81
REG_DATA0LOW     = 0x8C
REG_DATA1LOW     = 0x8E

GAIN_LOW  = ~0x10 & 0xff
GAIN_HIGH = 0x10 & 0xff

TIMING_13_7 = 0x00
TIMING_101  = 0x01
TIMING_402  = 0x02

TIMING_LIMITS = [0x13b7, 0x9139, 0xffff]
TIMING_RATIOS = [1/0.034, 1/0.252, 1.]

ERROR_OS      = -1
ERROR_TIMEOUT = -2
ERROR_LIGHT   = -3
ERROR_DARK    = -4
ERROR_UNKNOWN = -5

class APDS_9301:

	def init(self, address, gain, timing):
		# save config values
		self.address = address
		self.gain = gain
		self.timing = timing

		# I²C connection
		self.bus = smbus.SMBus(1)

		try:
			# enable power
			self.bus.write_byte_data(self.address, REG_CONTROL, 3)

			t_start = time.time()

			# wait for power-up
			while ((self.bus.read_byte_data(self.address, REG_CONTROL) & 0x03) == 0) \
				or (self.bus.read_word_data(self.address, REG_DATA1LOW) == 0):

				time.sleep(0.01)
				if (time.time() - t_start > 2):
					return ERROR_TIMEOUT

			# rad current config
			v = self.bus.read_byte_data(self.address, REG_TIMING)

			# cledar current timings: manual and integration
			v &= 0xf4

			# set gain
			if gain == GAIN_LOW:
				v &= GAIN_LOW
			else:
				v |= GAIN_HIGH;

			# set integration timing
			v |= timing

			# send
			self.bus.write_byte_data(self.address, REG_TIMING, v)

		except OSError as e:
			return ERROR_OS

		return 0

	def disable(self):
		# disable power
		self.bus.write_byte_data(self.address, REG_CONTROL, 0)

	def acquire(self):
		ch0 = self.bus.read_word_data(self.address, REG_DATA0LOW)
		ch1 = self.bus.read_word_data(self.address, REG_DATA1LOW)

		if ch1 >= TIMING_LIMITS[self.timing] or ch0 >= TIMING_LIMITS[self.timing]:
			return ERROR_LIGHT

		if ch0 == 0 or ch1 == 0:
			return ERROR_DARK

		ratio = ch1/ch0

		if ratio <= 0.50:
			result = (0.0304 * ch0) - (0.062 * ch0 * ((ch1/ch0)**1.4 ))
		elif ratio <= 0.61:
			result = (0.0224 * ch0) - (0.031 * ch1)
		elif ratio <= 0.80:
			result = (0.0128 * ch0) - (0.0153 * ch1)
		elif ratio <= 1.30:
			result = (0.00146 * ch0) - (0.00112 * ch1)
		else:
			return ERROR_UNKNOWN

		if self.gain == GAIN_LOW:
			result *= 16

		result *= TIMING_RATIOS[self.timing]

		return result
