#ifndef YURS_HARDWARE_CAN_UTIL_HPP
#define YURS_HARDWARE_CAN_UTIL_HPP

#include <stdint.h>

namespace yurs_hardware {
	template<typename T>
	void write_buffer(uint8_t *buf, T val);

	template<typename T>
	T read_buffer(const uint8_t *buf);

	template<>
	inline void write_buffer<uint32_t>(uint8_t *buf, uint32_t val) {
		buf[0] = (val >> 24) & 0xff;
		buf[1] = (val >> 16) & 0xff;
		buf[2] = (val >> 8) & 0xff;
		buf[3] = val & 0xff;
	}

	template<>
	inline uint32_t read_buffer<uint32_t>(const uint8_t *buf) {
		return
			((buf[0] & 0xff) << 24) |
			((buf[1] & 0xff) << 16) |
			((buf[2] & 0xff) << 8) |
			(buf[3] & 0xff);
	}

	template<>
	inline void write_buffer<int32_t>(uint8_t *buf, int32_t val) {
		write_buffer<uint32_t>(buf, val);
	}

	template<>
	inline int32_t read_buffer<int32_t>(const uint8_t *buf) {
		return read_buffer<uint32_t>(buf);
	}

	template<>
	inline void write_buffer<float>(uint8_t *buf, float val) {
		union {
			uint32_t u32;
			float f32;
		} v;
		v.f32 = val;
		write_buffer<uint32_t>(buf, v.u32);
	}

	template<>
	inline float read_buffer<float>(const uint8_t *buf) {
		union {
			uint32_t u32;
			float f32;
		} v;
		v.f32 = read_buffer<uint32_t>(buf);
		return v.u32;
	}

	template<>
	inline void write_buffer<uint16_t>(uint8_t *buf, uint16_t val) {
		buf[0] = (val >> 8) & 0xff;
		buf[1] = val & 0xff;
	}

	template<>
	inline uint16_t read_buffer<uint16_t>(const uint8_t *buf) {
		return
			((buf[0] & 0xff) << 8) |
			(buf[1] & 0xff);
	}

	template<>
	inline void write_buffer<int16_t>(uint8_t *buf, int16_t val) {
		return write_buffer<uint16_t>(buf, val);
	}

	template<>
	inline int16_t read_buffer<int16_t>(const uint8_t *buf) {
		return read_buffer<uint16_t>(buf);
	}

	template<>
	inline void write_buffer<uint8_t>(uint8_t *buf, uint8_t val) {
		buf[0] = val;
	}

	template<>
	inline uint8_t read_buffer<uint8_t>(const uint8_t *buf) {
		return buf[0];
	}

	template<>
	inline void write_buffer<int8_t>(uint8_t *buf, int8_t val) {
		return write_buffer<uint8_t>(buf, val);
	}

	template<>
	inline int8_t read_buffer<int8_t>(const uint8_t *buf) {
		return read_buffer<uint8_t>(buf);
	}
}

#endif
