#include "yurs_hardware/yurs_motor.hpp"

#include <cmath>
#include <boost/asio.hpp>
#include <linux/can.h>
#include <rclcpp/logging.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "can_util.hpp"

namespace asio = boost::asio;

using namespace yurs_hardware;

#define CAN_TYPE 2
#define CAN_MANUFACTURER 8

#define CAN_CLASS_CONTROL 0
#define CAN_INDEX_CONTROL_STOP 0
#define CAN_INDEX_CONTROL_RAW 1
#define CAN_INDEX_CONTROL_CURRENT 2

#define CAN_CLASS_TELEM 1
#define CAN_INDEX_TELEM_VOLTAGE 0
#define CAN_INDEX_TELEM_CURRENT 1
#define CAN_INDEX_TELEM_ENCODER 2
#define CAN_INDEX_TELEM_TEMP 3

struct frc_identifier {
	uint8_t type;
	uint8_t manufacturer;
	uint8_t api_class;
	uint8_t api_index;
	uint8_t devid;

	frc_identifier() : type(0), manufacturer(0), api_class(0), api_index(0), devid(0) {}

	frc_identifier(uint32_t can_id) :
		type((can_id >> 24) & 0x1F),
		manufacturer((can_id >> 16) & 0xFF),
		api_class((can_id >> 10) & 0x3F),
		api_index((can_id >> 6) & 0xF),
		devid((can_id >> 0) & 0x3F) {}

	uint32_t can_id() {
		uint32_t can_id = 0;
		can_id |= (type & 0x1F) << 24;
		can_id |= (manufacturer & 0xFF) << 16;
		can_id |= (api_class & 0x3F) << 10;
		can_id |= (api_index & 0xF) << 6;
		can_id |= (devid & 0x3F) << 0;
		return can_id;
	}
};

yurs_motor::motor_data::motor_data(std::string name, double cpr) : name(name), cpr(cpr) {}

yurs_motor::can_connection::can_connection(yurs_motor &interface) : _interface(interface) {
	int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (fd == -1) {
		throw std::system_error(errno, std::system_category());
	}

	ifreq ifr;
	strncpy(ifr.ifr_name, _interface._can_if.c_str(), IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = 0;

	if (ioctl(fd, SIOCGIFINDEX, &ifr) == -1) {
		throw std::system_error(errno, std::system_category());
	}

	sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) == -1) {
		throw std::system_error(errno, std::system_category());
	}

	_descriptor = std::make_shared<asio::posix::stream_descriptor>(*_interface._ctx, fd);
};

std::shared_ptr<yurs_motor::can_connection> yurs_motor::can_connection::can_connection::create(yurs_motor &interface) {
	return std::shared_ptr<can_connection>(new can_connection(interface));
}

void yurs_motor::can_connection::read() {
	_descriptor->async_read_some(asio::buffer(_frame.bytes), [ptr = shared_from_this()](boost::system::error_code ec, [[maybe_unused]] auto length) {
		auto frame = ptr->_frame.frame;
		ptr->read();

		if (ec) {
			ptr->read();
			// TODO: handle error
			return;
		}

		if (!(frame.can_id & CAN_EFF_FLAG)) return;

		frc_identifier ident(frame.can_id & CAN_EFF_MASK);
		if (ident.type != CAN_TYPE || ident.manufacturer != CAN_MANUFACTURER) return;

		auto it = ptr->_interface._joints.find(ident.devid);
		if (it == ptr->_interface._joints.end()) return;

		auto &data = it->second;

		if (ident.api_class == CAN_CLASS_TELEM) {
			if (ident.api_index == CAN_INDEX_TELEM_ENCODER) {
				if (std::isnan(data.cpr)) return;

				auto counts = read_buffer<uint32_t>(frame.data);
				data.input_position_shadow = 2 * M_PI * (counts / data.cpr);
			}
		}
	});
}

static void motor_current(can_frame *frame, uint8_t devid, float current) {
	frc_identifier ident;
	ident.type = CAN_TYPE;
	ident.manufacturer = CAN_MANUFACTURER;
	ident.api_class = CAN_CLASS_CONTROL;
	ident.api_index = CAN_INDEX_CONTROL_CURRENT;
	ident.devid = devid;

	frame->can_id = ident.can_id() | CAN_EFF_FLAG;
	frame->can_dlc = 4;
	write_buffer(frame->data, current);

	// TODO: remove this hack
	ident.api_index = CAN_INDEX_CONTROL_RAW;
	int16_t val = 65536 * current / 80.0;
	frame->can_dlc = 2;
	write_buffer(frame->data, val);
	frame->can_id = ident.can_id() | CAN_EFF_FLAG;
}

void yurs_motor::can_connection::write(std::shared_ptr<can_frame> frame) {
	void *ptr = frame.get();

	_descriptor->async_write_some(asio::buffer(ptr, sizeof(can_frame)), [ptr = shared_from_this(), frame = std::move(frame)](boost::system::error_code ec, [[maybe_unused]] auto length) {
		if (ec) {
			// TODO: handle error
			return;
		}
	});
}

yurs_motor::CallbackReturn yurs_motor::on_init(const HardwareInfo &hardware_info) {
	if (auto ret = hardware_interface::ActuatorInterface::on_init(hardware_info); ret != CallbackReturn::SUCCESS) {
		return ret;
	}

	const auto &hw_params = hardware_info.hardware_parameters;

	const auto can_if_it = hw_params.find("can_interface");
	if (can_if_it == hw_params.end()) {
		RCLCPP_ERROR(this->get_logger(), "Missing can_interface parameter");
		return CallbackReturn::ERROR;
	}
	_can_if = can_if_it->second;

	if (hardware_info.sensors.size() > 0) return CallbackReturn::ERROR;
	if (hardware_info.gpios.size() > 0) return CallbackReturn::ERROR;
	if (hardware_info.transmissions.size() > 0) return CallbackReturn::ERROR;

	for (const auto &joint : hardware_info.joints) {
		const auto devid_it = joint.parameters.find("device_id");
		if (devid_it == joint.parameters.end()) {
			RCLCPP_ERROR(this->get_logger(), "Joint is missing device_id parameter");
			return CallbackReturn::ERROR;
		}

		double cpr;
		const auto cpr_it = joint.parameters.find("counts_per_rev");
		if (cpr_it != joint.parameters.end()) {
			try {
				cpr = std::stod(cpr_it->second);
			} catch (...) {
				RCLCPP_ERROR(this->get_logger(), "Joint has invalid counts per rev: %s", cpr_it->second.c_str());
				return CallbackReturn::ERROR;
			}
		} else {
			cpr = std::numeric_limits<double>::quiet_NaN();
		}

		if (std::isnan(cpr)) {
			RCLCPP_WARN(this->get_logger(), "No counts per rev value for joint, position unavailable: %s", joint.name.c_str());
		}

		int devid;

		try {
			devid = std::stoi(devid_it->second);
		} catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Joint has invalid device ID: %s", devid_it->second.c_str());
			return CallbackReturn::ERROR;
		}

		if (!(devid >= 0 && devid < 64)) {
			RCLCPP_ERROR(this->get_logger(), "Joint device ID is out of range: %d", devid);
			return CallbackReturn::ERROR;
		}

		_joints.emplace(devid, motor_data(joint.name, cpr));

	}

	return CallbackReturn::SUCCESS;
}

std::vector<yurs_motor::StateInterface> yurs_motor::export_state_interfaces() {
	std::vector<hardware_interface::StateInterface> interfaces;

	for (auto &entry : _joints) {
		auto &data = entry.second;
		if (!std::isnan(data.cpr)) {
			RCLCPP_INFO(this->get_logger(), "Exporting position state interface: %s", data.name.c_str());
			interfaces.emplace_back(data.name, hardware_interface::HW_IF_POSITION, &data.input_position);
		}
	}

	return interfaces;
}

std::vector<yurs_motor::CommandInterface> yurs_motor::export_command_interfaces() {
	std::vector<hardware_interface::CommandInterface> interfaces;

	for (auto &entry : _joints) {
		auto &data = entry.second;
		RCLCPP_INFO(this->get_logger(), "Exporting effort command interface: %s", data.name.c_str());
		interfaces.emplace_back(data.name, hardware_interface::HW_IF_EFFORT, &data.output_effort);
	}

	return interfaces;
}

yurs_motor::CallbackReturn yurs_motor::on_configure(const State &) {
	_ctx = std::make_unique<asio::io_context>();

	auto can_ptr = can_connection::create(*this);
	_can = can_ptr;

	can_ptr->read();

	std::thread th([this]() {
		_ctx->run();
	});
	th.detach();

	return CallbackReturn::SUCCESS;
}

yurs_motor::return_type yurs_motor::read(const rclcpp::Time &, const rclcpp::Duration &) {
	for (auto &entry : _joints) {
		auto &data = entry.second;
		data.input_position = data.input_position_shadow;
	}
	return return_type::OK;
}

yurs_motor::return_type yurs_motor::write(const rclcpp::Time &, const rclcpp::Duration &) {
	auto can_ptr = _can.lock();
	if (!can_ptr) return return_type::ERROR;

	for (auto &entry : _joints) {
		auto &data = entry.second;
		data.output_effort_shadow = data.output_effort;

		auto frame = std::make_shared<can_frame>();
		motor_current(frame.get(), entry.first, data.output_effort_shadow);
		can_ptr->write(std::move(frame));
	}

	return return_type::OK;
}


yurs_motor::CallbackReturn yurs_motor::on_cleanup(const State &) {
	_ctx.reset();
	return CallbackReturn::SUCCESS;
}

yurs_motor::CallbackReturn yurs_motor::on_shutdown(const State &) {
	_ctx.reset();
	return CallbackReturn::SUCCESS;
}

yurs_motor::CallbackReturn yurs_motor::on_activate(const State &) {
	return CallbackReturn::SUCCESS;
}

yurs_motor::CallbackReturn yurs_motor::on_deactivate(const State &) {
	return CallbackReturn::SUCCESS;
}

yurs_motor::CallbackReturn yurs_motor::on_error(const State &) {
	_ctx.reset();
	return CallbackReturn::SUCCESS;
}

yurs_motor::~yurs_motor() {

}

rclcpp::Logger yurs_motor::get_logger() {
	return rclcpp::get_logger(this->get_name());
}

PLUGINLIB_EXPORT_CLASS(yurs_hardware::yurs_motor, hardware_interface::ActuatorInterface)
