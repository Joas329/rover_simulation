#ifndef YURS_HARDWARE_MOTOR_HPP
#define YURS_HARDWARE_MOTOR_HPP

#include <thread>
#include <linux/can.h>
#include <hardware_interface/actuator_interface.hpp>
#include <rclcpp/logger.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

namespace yurs_hardware {
	class yurs_motor : public hardware_interface::ActuatorInterface {
		using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
		using State = rclcpp_lifecycle::State;
		using HardwareInfo = hardware_interface::HardwareInfo;
		using StateInterface = hardware_interface::StateInterface;
		using CommandInterface = hardware_interface::CommandInterface;
		using return_type = hardware_interface::return_type;

		struct motor_data {
			std::string name;

			double cpr;

			double output_effort;
			double output_effort_shadow;

			double input_position;
			double input_position_shadow;

			motor_data(std::string name, double cpr);
		};

		struct can_connection : public std::enable_shared_from_this<can_connection> {
			yurs_motor &_interface;
			std::shared_ptr<boost::asio::posix::stream_descriptor> _descriptor;

			union {
				can_frame frame;
				char bytes[sizeof(can_frame)];
			} _frame;

			can_connection(yurs_motor &interface);

		public:
			[[nodiscard]]
			static std::shared_ptr<can_connection> create(yurs_motor &interface);

			void read();

			void write(std::shared_ptr<can_frame> frame);
		};

		std::string _can_if;
		std::unordered_map<int, motor_data> _joints;
		std::unique_ptr<boost::asio::io_context> _ctx;
		std::weak_ptr<can_connection> _can;

		rclcpp::Logger get_logger();

	public:
		~yurs_motor() override;

		CallbackReturn on_configure(const State &previous_state) override;
		CallbackReturn on_cleanup(const State &previous_state) override;
		CallbackReturn on_shutdown(const State &previous_state) override;
		CallbackReturn on_activate(const State &previous_state) override;
		CallbackReturn on_deactivate(const State &previous_state) override;
		CallbackReturn on_error(const State &previous_state) override;
		CallbackReturn on_init(const HardwareInfo &hardware_info) override;
		std::vector<StateInterface> export_state_interfaces() override;
		std::vector<CommandInterface> export_command_interfaces() override;
		return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
		return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
	};
}

#endif
