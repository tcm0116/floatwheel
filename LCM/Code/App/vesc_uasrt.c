#include "vesc_uasrt.h"
#include "flag_bit.h"
#include "eeprom.h"
#include "task.h"

uint8_t VESC_RX_Buff[80];
uint8_t VESC_RX_Flag = 0;

#ifdef GTV
#define FIRMWARE_ID "GTV_2_1_2"
#endif
#ifdef XRV
#define FIRMWARE_ID "XRV_2_1_2"
#endif
#ifdef PINTV
#define FIRMWARE_ID "PintV_2_1_2"
#endif
#ifdef ADV
#define FIRMWARE_ID "ADV_2_1_2"
#endif

// Access ADC values here to determine riding state
extern float ADC1_Val, ADC2_Val;

dataPackage data;
lcmConfig_t lcmConfig;

/**************************************************
 * @brie   :Send_Pack_Data()
 * @note   :����һ������
 * @param  :payload Ҫ�������ݰ�����ʼ��ַ
 *          len ���ݰ�����
 * @retval :��
 **************************************************/
void Send_Pack_Data(uint8_t *payload,uint16_t len) 
{
	uint8_t protocol_buff[40]; //���ͻ�����
	uint8_t count = 0;
	uint16_t crcpayload = crc16(payload, len);  //����У�� 
	
	/*
		Э���ʽ
	
		��ʼ�ֽڣ�һ���ֽڣ� + ���ݰ����ȣ�һ���������ֽڣ� + ���ݰ���N���ֽڣ� + У�飨�����ֽڣ� + ֹͣ�ֽڣ�һ���ֽڣ�
	
		��ʼ�ֽ�:	0x02���ݰ�����1-256���ֽ�
					0x03���ݰ����ȳ���256���ֽ�
	
		���ݰ�����: ��ʼ�ֽ�0x02 ���ݰ�ռһ���ֽ�
					��ʼ�ֽ�0x03 ���ݰ�ռ�����ֽ�
	
		���ݰ�:  	���ݰ���һ���ֽ�Ϊ���ݰ�ID
	
		У��:		CRCУ�� �����ֽ� 
		
		ֹͣ�ֽ�:   �̶�0x03
	
	*/
	
	if (len <= 32) //���ݰ����Ȳ�����256���ֽ�
	{
		protocol_buff[count++] = 2;
		protocol_buff[count++] = len;
	}
	else //���ݰ����ȴ���256���ֽ�
	{
		// no need to support extra long messages since we don't use such messages
		return;
		//protocol_buff[count++] = 3;
		//protocol_buff[count++] = (uint8_t)(len >> 8);
		//protocol_buff[count++] = (uint8_t)(len & 0xFF);
	}

	memcpy(&protocol_buff[count], payload, len);  //�����ݰ����Ƶ�Э����

	count += len;
	protocol_buff[count++] = (uint8_t)(crcpayload >> 8);
	protocol_buff[count++] = (uint8_t)(crcpayload & 0xFF);
	protocol_buff[count++] = 3;
	
	USART1_Send_Bytes(protocol_buff,count);
}

void buffer_append_int16(uint8_t* buffer, int16_t number, uint8_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t* buffer, float number, uint8_t scale, uint8_t *index) {
	buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

/**************************************************
 * @brie   :Get_Vesc_Pack_Data()
 * @note   :��ȡһ������
 * @param  :id ���ݰ�id
 * @retval :��
 **************************************************/
void Get_Vesc_Pack_Data(COMM_PACKET_ID id)
{
	uint8_t command[32];
	int len = 1;
	
	command[0] = id;
	
	if (id == COMM_CUSTOM_APP_DATA) {
		command[1] = 101;
		command[2] = 24; // FLOAT_COMMAND_POLL
		len = 3;
		if (!lcmConfig.isSet) {
			// write firmware id string to command
			int firmwareIdSize = sizeof(FIRMWARE_ID);
			memcpy(&command[3], FIRMWARE_ID, firmwareIdSize);
			len += firmwareIdSize;
		}
	}

	if (id == COMM_CHARGE_INFO) {
		command[0] = COMM_CUSTOM_APP_DATA;
		command[1] = 101;
		command[2] = 28; 											// FLOAT_COMMAND_CHARGESTATE
 		command[3] = 151; 											// -charging: 1/0 aka true/false
#ifdef HAS_CHARGING
 		command[4] = Charge_Flag == 2 ? 1: 0; 						// -charging: 1/0 aka true/false
#else
 		command[4] = 0; 						                    // -charging: 1/0 aka true/false
		const float Charge_Voltage = 0;
		const float Charge_Current = 0;
#endif
		uint8_t ind = 5;
		buffer_append_float16(command, Charge_Voltage, 10, &ind); 	// -voltage: 16bit float divided by 10
		buffer_append_float16(command, Charge_Current, 10, &ind); 	// -current: 16bit float divided by 10
		len = 9;
	}

	if (id == COMM_CUSTOM_DEBUG) {
		command[0] = COMM_CUSTOM_APP_DATA;
		command[1] = 101;
		command[2] = 99; // FLOAT_COMMAND_LCM_DEBUG
		command[3] = Power_Flag;
#ifdef HAS_CHARGING
		command[4] = Charge_Flag;
#else
		command[4] = 0;
#endif
		command[5] = data.dutyCycleNow;
#ifdef HAS_WS2812
		command[6] = WS2812_Display_Flag;
		command[7] = WS2812_Flag;
#else
		command[6] = 0;
		command[7] = 0;
#endif        
		command[8] = Shutdown_Time_M;
		command[9] = Shutdown_Time_S / 1000;
		command[10] = GPIOC->IDR;
		len = 11;
	}
	
	Send_Pack_Data(command, len);
}

/**************************************************
 * @brie   :buffer_get_int16()
 * @note   :�����������ֽ�ƴһ��int16_t
 * @param  :buffer��ַ  index��ַƫ��
 * @retval :��
 **************************************************/
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}
/**************************************************
 * @brie   :buffer_get_uint16()
 * @note   :�����������ֽ�ƴһ��uint16_t
 * @param  :buffer��ַ  index��ַƫ��
 * @retval :��
 **************************************************/
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}
/**************************************************
 * @brie   :buffer_get_int32()
 * @note   :�������ĸ��ֽ�ƴһ��int32_t
 * @param  :buffer��ַ  index��ַƫ��
 * @retval :��
 **************************************************/
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}
/**************************************************
 * @brie   :buffer_get_uint32()
 * @note   :�������ĸ��ֽ�ƴһ��uint32_t
 * @param  :buffer��ַ  index��ַƫ��
 * @retval :��
 **************************************************/
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}
/**************************************************
 * @brie   :buffer_get_float16()
 * @note   :�����������ֽ�ƴһ��float
 * @param  :buffer��ַ  index��ַƫ��  scale��ĸ
 * @retval :��
 **************************************************/
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
	return (float)buffer_get_int16(buffer, index) / scale;
}
/**************************************************
 * @brie   :buffer_get_float32()
 * @note   :�������ĸ��ֽ�ƴһ��float
 * @param  :buffer��ַ  index��ַƫ��	scale��ĸ
 * @retval :��
 **************************************************/
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
	return (float)buffer_get_int32(buffer, index) / scale;
}

void Process_Command(uint8_t command, uint8_t data)
{
	switch (command) {
		/*case HEADLIGHT_BRIGHTNESS:
			lcmConfig.headlightBrightness = data;
			return;
		case HEADLIGHT_IDLE_BRIGHTNESS:
			lcmConfig.headlightIdleBrightness = data;
			return;
		case STATUSBAR_BRIGHTNESS:
			lcmConfig.statusbarBrightness = data;
			return;*/
		/*case STATUS_BAR_IDLE_MODE:
			if (data != lcmConfig.statusBarIdleMode) {
				lcmConfig.statusBarIdleMode = data;
				EEPROM_WriteByte(STATUS_BAR_IDLE_MODE, data);
			}
			return;
		case BOOT_ANIMATION:
			if (data != lcmConfig.bootAnimation) {
				lcmConfig.bootAnimation = data;
				EEPROM_WriteByte(BOOT_ANIMATION, data);
			}
			return;
		case DUTY_BEEP:
			if (data != lcmConfig.dutyBeep) {
				lcmConfig.dutyBeep = data;
				//EEPROM_WriteByte(DUTY_BEEP, data);
			}
			return;*/
		case POWER_OFF:
			lcmConfig.boardOff = data == 1;
			return;
		/*case CHARGE_CUTOFF:
			//lcmConfig.chargeCutoffVoltage = data;
			return;
		case AUTO_SHUTDOWN:
			if (data != lcmConfig.autoShutdownTime) {
				lcmConfig.autoShutdownTime = data;
				//EEPROM_WriteByte(AUTO_SHUTDOWN, data);
			}
			return;*/
		case FACTORY_RESET:
			if (data == 1) {
				//EEPROM_EraseAll();
			}
			return;
		case DEBUG:
			lcmConfig.debug = data == 1;
			return;
		}
}

/**************************************************
 * @brie   :Protocol_Parse()
 * @note   :Э�����
 * @param  :message ���յ����ݵ���ʼ��ַ
 * @retval :0 �����ɹ� 1����ʧ��
 **************************************************/
uint8_t Protocol_Parse(uint8_t * message)
{
	uint8_t  start = 0;
	uint8_t *pdata;
	uint16_t counter = 0;
	uint16_t len = 0; 
	uint16_t crcpayload;
	uint8_t id;
	int32_t ind = 0;
	
	start = message[counter++];
	
	switch(start)
	{
		case 0x02:
			len = message[counter++];
		break;
		
		//case 0x03:
			// we don't support/expect long messages, return error
		default:
			return 1;
		break;
		
	}

	if (len > sizeof(VESC_RX_Buff) - 4)
	{
		// The indicated length of the message is greater than the buffer size
		return 1;
	}
	
	crcpayload = crc16(&message[counter], len);
	
	if(crcpayload != (((uint16_t)message[counter+len])<<8|
					 ((uint16_t)message[counter+len+1])))
	{
		return 1; //crc is wrong
	}
	
	id = message[counter++];
	pdata = &message[counter];  
	
	switch(id)
	{
		case COMM_GET_VALUES: 

			ind += 8;
			data.avgInputCurrent 	= buffer_get_float32(pdata, 100.0, &ind); // negative input current implies braking
			ind += 8; // Skip id/iq currents
			data.dutyCycleNow 		= buffer_get_float16(pdata, 10.0, &ind);	// duty as value 0..100
			data.rpm 				= buffer_get_int32(pdata, &ind);
			data.inpVoltage 		= buffer_get_float16(pdata, 10.0, &ind);

			if ((data.rpm > 100) || (data.rpm < -100) || (data.avgInputCurrent > 1) || (data.avgInputCurrent < -1)) {
				data.state = RUNNING;
			}
			else {
				// Use this fault as a placeholder (we only care that the board is stopped anyways)
				data.state = FAULT_STARTUP;
			}
			data.isOldPackage = true;
		
		break;
		
		case COMM_CUSTOM_APP_DATA:

		  if (len < 12) {
				break;
			}
		  	uint8_t magicnr = pdata[ind++];
		  	uint8_t floatcmd = pdata[ind++];
			if ((magicnr != 101) || (floatcmd != FLOAT_COMMAND_LCM_POLL)) {
				break;
			}
			data.floatPackageSupported = true;
			uint8_t state = pdata[ind++];
			data.state = state & 0xF;
			//data.switchstate = (state >> 4) & 0x7;
			data.isHandtest = (state & 0x80) > 0;
			data.fault = pdata[ind++];
			if ((data.state == RUNNING) || (data.state == RUNNING_TILTBACK) || (data.state == RUNNING_WHEELSLIP)) {
				data.dutyCycleNow = pdata[ind++];
				data.pitch = 0;
			}
			else {
				data.pitch = pdata[ind++];
				data.dutyCycleNow = 0;
			}
			data.rpm = buffer_get_float16(pdata, 1.0, &ind);
			data.avgInputCurrent = buffer_get_float16(pdata, 1.0, &ind);

			float v = buffer_get_float16(pdata, 10.0, &ind);
			if (data.inpVoltage < BATTERY_STRING * 2.5)
				data.inpVoltage = v;
			else
				data.inpVoltage = data.inpVoltage * 0.9 + 0.1 * v;

			if ((len >= ind + 3)) {
				// Float package is 0-100 range. Adjust as needed
				uint8_t headlightBrightness = pdata[ind++] * 255/100;
				uint8_t headlightIdleBrightness = pdata[ind++] * 255/100;
				uint8_t statusbarBrightness = pdata[ind++] * 255/100;

				// Only set isSet if something changed
				// Allows use of the power button to go back to default behaviour
				if (headlightBrightness != lcmConfig.headlightBrightness || headlightIdleBrightness != lcmConfig.headlightIdleBrightness || statusbarBrightness != lcmConfig.statusbarBrightness) {
					lcmConfig.isSet = true;
				}

				lcmConfig.headlightBrightness = headlightBrightness;
				lcmConfig.headlightIdleBrightness = headlightIdleBrightness;
				lcmConfig.statusbarBrightness = statusbarBrightness;

				// Process generic command/config
				while (len >= ind + 2) {
					uint8_t command = pdata[ind++];
					uint8_t data = pdata[ind++];
					Process_Command(command, data);
				}
			}
	}
	if (data.rpm > 100)
		data.isForward = data.state != RUNNING_UPSIDEDOWN;
	if (data.rpm < -100)
		data.isForward = data.state == RUNNING_UPSIDEDOWN;
	if (data.state > RUNNING_FLYWHEEL)
		data.isForward = true;

	return 0;
}

