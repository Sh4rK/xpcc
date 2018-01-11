/* Copyright (c) 2018, Roboterclub Aachen e.V., Antal Szabó
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 * ------------------------------------------------------------------------ */

//
// NUCLEO-F446RE
// Nucleo kit for STM32F446RE
//

#ifndef XPCC_STM32_NUCLEO_F446RE_HPP
#define XPCC_STM32_NUCLEO_F446RE_HPP

#include <xpcc/architecture/platform.hpp>
#include <xpcc/debug/logger.hpp>
#define XPCC_BOARD_HAS_LOGGER

using namespace xpcc::stm32;


namespace Board
{

// STM32F446RE running at 180MHz generated from the internal 16MHz crystal
// Dummy clock for devices
struct systemClock
{
	static constexpr uint32_t Frequency = MHz180;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency / 4;
	static constexpr uint32_t Apb2 = Frequency / 2;

	static constexpr uint32_t Adc1 = Apb2;
	static constexpr uint32_t Adc2 = Apb2;
	static constexpr uint32_t Adc3 = Apb2;

	static constexpr uint32_t Spi1 = Apb2;
	static constexpr uint32_t Spi2 = Apb1;
	static constexpr uint32_t Spi3 = Apb1;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;
	static constexpr uint32_t Usart3 = Apb1;
	static constexpr uint32_t Usart4 = Apb1;
	static constexpr uint32_t Usart5 = Apb1;
	static constexpr uint32_t Usart6 = Apb2;

	static constexpr uint32_t Can1 = Apb1;
	static constexpr uint32_t Can2 = Apb1;

	static constexpr uint32_t I2c1 = Apb1;
	static constexpr uint32_t I2c2 = Apb1;
	static constexpr uint32_t I2c3 = Apb1;

	static constexpr uint32_t Apb1Timer = Apb1 * 2;
	static constexpr uint32_t Apb2Timer = Apb2 * 2;
	static constexpr uint32_t Timer1 = Apb2Timer;
	static constexpr uint32_t Timer2 = Apb1Timer;
	static constexpr uint32_t Timer3 = Apb1Timer;
	static constexpr uint32_t Timer4 = Apb1Timer;
	static constexpr uint32_t Timer5 = Apb1Timer;
	static constexpr uint32_t Timer6 = Apb1Timer;
	static constexpr uint32_t Timer7 = Apb1Timer;
	static constexpr uint32_t Timer8 = Apb2Timer;
	static constexpr uint32_t Timer9 = Apb2Timer;
	static constexpr uint32_t Timer10 = Apb2Timer;
	static constexpr uint32_t Timer11 = Apb2Timer;
	static constexpr uint32_t Timer12 = Apb1Timer;
	static constexpr uint32_t Timer13 = Apb1Timer;
	static constexpr uint32_t Timer14 = Apb1Timer;

	static bool inline
	enable()
	{
		ClockControl::enableInternalClock(); // 16MHz
		ClockControl::enablePll(
			ClockControl::PllSource::InternalClock,
			8,   // 16MHz / M=8 -> 2MHz
			180, // 2MHz * N=180 -> 360MHz
			2,   // 360MHz / P=2 -> 180MHz = F_cpu
			8    // 360MHz / Q=8 -> 45MHz = F_usb => bad for USB (it requires 48MHz)
		);
		// set flash latency for 180MHz
		ClockControl::setFlashLatency(Frequency);
		// switch system clock to PLL output
		ClockControl::enableSystemClock(ClockControl::SystemClockSource::Pll);
		ClockControl::setAhbPrescaler(ClockControl::AhbPrescaler::Div1);
		// APB1 has max. 45MHz
		ClockControl::setApb1Prescaler(ClockControl::Apb1Prescaler::Div4);
		// APB2 has max. 90MHz
		ClockControl::setApb2Prescaler(ClockControl::Apb2Prescaler::Div2);
		// update frequencies for busy-wait delay functions
		xpcc::clock::fcpu = Frequency;
		xpcc::clock::fcpu_kHz = Frequency / 1000;
		xpcc::clock::fcpu_MHz = Frequency / 1000000;
		xpcc::clock::ns_per_loop = ::round(3000.f / (Frequency / 1000000));

		return true;
	}
};

// Arduino Footprint
#include "../nucleo64_arduino.hpp"

using Button = xpcc::GpioInverted<GpioInputC13>;
using LedD13 = D13;

using Leds = xpcc::SoftwareGpioPort<LedD13>;

namespace stlink
{
using Rx = GpioInputA3;
using Tx = GpioOutputA2;
using Uart = Usart2;
}

// Create an IODeviceWrapper around the Uart Peripheral we want to use
using LoggerDevice = xpcc::IODeviceWrapper<stlink::Uart, xpcc::IOBuffer::BlockIfFull>;

inline void
initialize()
{
	systemClock::enable();
	xpcc::cortex::SysTickTimer::initialize<systemClock>();

	stlink::Tx::connect(stlink::Uart::Tx);
	stlink::Rx::connect(stlink::Uart::Rx, Gpio::InputType::PullUp);
	stlink::Uart::initialize<systemClock, xpcc::Uart::Baudrate::B115200>(12);

	Button::setInput();
	Button::setInputTrigger(Gpio::InputTrigger::RisingEdge);
	Button::enableExternalInterrupt();
//	Button::enableExternalInterruptVector(12);
}

}

#endif // XPCC_STM32_NUCLEO_F446RE_HPP
