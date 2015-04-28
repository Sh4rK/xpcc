// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_SSD1306_HPP
#	error	"Don't include this file directly, use 'ssd1306.hpp' instead!"
#endif

template < class I2cMaster >
xpcc::Ssd1306<I2cMaster>::Ssd1306(uint8_t address)
:	I2cDevice<I2cMaster, 2, ssd1306::DataTransmissionAdapter>(address),
	commandBuffer{0x80, 0, 0x80, 0, 0x80, 0, 0x80, 0, 0x80, 0, 0x80, 0, 0x80, 0}
{
	this->transaction.setCommandBuffer(commandBuffer);
}

// ----------------------------------------------------------------------------
// MARK: - Tasks
template < class I2cMaster >
xpcc::co::Result<bool>
xpcc::Ssd1306<I2cMaster>::initialize()
{
	CO_BEGIN();

	commandBuffer[11] = true;
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetDisplayOff));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetDisplayClockDivideRatio, 0xF0));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetMultiplexRatio, 0x3F));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetDisplayOffset, 0x00));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetDisplayStartLine | 0x00));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetChargePump, 0x14));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetMemoryMode, 0x01));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetSegmentRemap127));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetComOutputScanDirectionDecrement));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetComPins, 0x12));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetContrastControl, 0xCE));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetPreChargePeriod, 0xF1));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetV_DeselectLevel, 0x40));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetEntireDisplayResumeToRam));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetNormalDisplay));
//	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetColumnAddress, 0, 127));
//	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetPageAddress, 0, 7));
	commandBuffer[11] &= CO_CALL(writeCommand(Command::SetDisplayOn));

	CO_END_RETURN(commandBuffer[11]);
}

// ----------------------------------------------------------------------------
template < class I2cMaster >
xpcc::co::Result<void>
xpcc::Ssd1306<I2cMaster>::startWriteDisplay()
{
	CO_BEGIN();

	CO_WAIT_UNTIL( this->transaction.configureDisplayWrite(buffer, 1024) and this->startTransaction() );

	CO_END();
}

template < class I2cMaster >
xpcc::co::Result<bool>
xpcc::Ssd1306<I2cMaster>::writeDisplay()
{
	CO_BEGIN();

	CO_CALL(startWriteDisplay());

	CO_WAIT_WHILE(this->isTransactionRunning());

	CO_END_RETURN(this->wasTransactionSuccessful());
}

template < class I2cMaster >
xpcc::co::Result<bool>
xpcc::Ssd1306<I2cMaster>::setRotation(Rotation rotation)
{
	CO_BEGIN();

	if ( CO_CALL(writeCommand((rotation == Rotation::Normal) ?
			Command::SetSegmentRemap127 : Command::SetSegmentRemap0)) )
	{
		CO_RETURN_CALL(writeCommand((rotation == Rotation::Normal) ?
				Command::SetComOutputScanDirectionDecrement : Command::SetComOutputScanDirectionIncrement));
	}

	CO_END_RETURN(false);
}

template < class I2cMaster >
xpcc::co::Result<bool>
xpcc::Ssd1306<I2cMaster>::configureScroll(uint8_t origin, uint8_t size,
		ScrollDirection direction, ScrollStep steps)
{
	CO_BEGIN();

	if (!CO_CALL(disableScroll()))
		CO_RETURN(false);

	{
		uint8_t beginY = (origin > 7) ? 7 : origin;

		uint8_t endY = ((origin + size) > 7) ? 7 : (origin + size);
		if (endY < beginY) endY = beginY;

		commandBuffer[1] = static_cast<uint8_t>(direction);
		commandBuffer[3] = 0x00;
		commandBuffer[5] = beginY;
		commandBuffer[7] = i(steps);
		commandBuffer[9] = endY;
		commandBuffer[11] = 0x00;
		commandBuffer[13] = 0xFF;
	}

	CO_WAIT_UNTIL( startTransactionWithLength(14) );

	CO_WAIT_WHILE(this->isTransactionRunning());

	CO_END_RETURN(this->wasTransactionSuccessful());
}

// ----------------------------------------------------------------------------
// MARK: write command
template < class I2cMaster >
xpcc::co::Result<bool>
xpcc::Ssd1306<I2cMaster>::writeCommand(uint8_t command)
{
	CO_BEGIN();

	commandBuffer[1] = command;
	CO_WAIT_UNTIL( startTransactionWithLength(2) );

	CO_WAIT_WHILE(this->isTransactionRunning());

	CO_END_RETURN(this->wasTransactionSuccessful());
}

template < class I2cMaster >
xpcc::co::Result<bool>
xpcc::Ssd1306<I2cMaster>::writeCommand(uint8_t command, uint8_t data)
{
	CO_BEGIN();

	commandBuffer[1] = command;
	commandBuffer[3] = data;
	CO_WAIT_UNTIL( startTransactionWithLength(4) );

	CO_WAIT_WHILE(this->isTransactionRunning());

	CO_END_RETURN(this->wasTransactionSuccessful());
}

template < class I2cMaster >
xpcc::co::Result<bool>
xpcc::Ssd1306<I2cMaster>::writeCommand(uint8_t command, uint8_t data1, uint8_t data2)
{
	CO_BEGIN();

	commandBuffer[1] = command;
	commandBuffer[3] = data1;
	commandBuffer[5] = data2;
	CO_WAIT_UNTIL( startTransactionWithLength(6) );

	CO_WAIT_WHILE(this->isTransactionRunning());

	CO_END_RETURN(this->wasTransactionSuccessful());
}

// ----------------------------------------------------------------------------
template < class I2cMaster >
bool
xpcc::Ssd1306<I2cMaster>::startTransactionWithLength(uint8_t length)
{
	return this->startWrite(commandBuffer, length);
}
