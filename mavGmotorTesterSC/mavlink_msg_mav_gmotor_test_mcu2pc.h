// MESSAGE MAV_GMOTOR_TEST_MCU2PC PACKING

#define MAVLINK_MSG_ID_MAV_GMOTOR_TEST_MCU2PC 151

typedef struct __mavlink_mav_gmotor_test_mcu2pc_t
{
 float motor_temp; ///<  nhiet do dong co 
 uint8_t motor_id; ///<  dia chi dong co 
 uint8_t motor_status; ///<  trang thay dong co 
} mavlink_mav_gmotor_test_mcu2pc_t;

#define MAVLINK_MSG_ID_MAV_GMOTOR_TEST_MCU2PC_LEN 6
#define MAVLINK_MSG_ID_151_LEN 6



#define MAVLINK_MESSAGE_INFO_MAV_GMOTOR_TEST_MCU2PC { \
	"MAV_GMOTOR_TEST_MCU2PC", \
	3, \
	{  { "motor_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mav_gmotor_test_mcu2pc_t, motor_temp) }, \
         { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mav_gmotor_test_mcu2pc_t, motor_id) }, \
         { "motor_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mav_gmotor_test_mcu2pc_t, motor_status) }, \
         } \
}


/**
 * @brief Pack a mav_gmotor_test_mcu2pc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  dia chi dong co 
 * @param motor_status  trang thay dong co 
 * @param motor_temp  nhiet do dong co 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_gmotor_test_mcu2pc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t motor_id, uint8_t motor_status, float motor_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_float(buf, 0, motor_temp);
	_mav_put_uint8_t(buf, 4, motor_id);
	_mav_put_uint8_t(buf, 5, motor_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_mav_gmotor_test_mcu2pc_t packet;
	packet.motor_temp = motor_temp;
	packet.motor_id = motor_id;
	packet.motor_status = motor_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_GMOTOR_TEST_MCU2PC;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 245);
}

/**
 * @brief Pack a mav_gmotor_test_mcu2pc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  dia chi dong co 
 * @param motor_status  trang thay dong co 
 * @param motor_temp  nhiet do dong co 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_gmotor_test_mcu2pc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t motor_id,uint8_t motor_status,float motor_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_float(buf, 0, motor_temp);
	_mav_put_uint8_t(buf, 4, motor_id);
	_mav_put_uint8_t(buf, 5, motor_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_mav_gmotor_test_mcu2pc_t packet;
	packet.motor_temp = motor_temp;
	packet.motor_id = motor_id;
	packet.motor_status = motor_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAV_GMOTOR_TEST_MCU2PC;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 245);
}

/**
 * @brief Encode a mav_gmotor_test_mcu2pc struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_gmotor_test_mcu2pc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_gmotor_test_mcu2pc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_gmotor_test_mcu2pc_t* mav_gmotor_test_mcu2pc)
{
	return mavlink_msg_mav_gmotor_test_mcu2pc_pack(system_id, component_id, msg, mav_gmotor_test_mcu2pc->motor_id, mav_gmotor_test_mcu2pc->motor_status, mav_gmotor_test_mcu2pc->motor_temp);
}

/**
 * @brief Send a mav_gmotor_test_mcu2pc message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  dia chi dong co 
 * @param motor_status  trang thay dong co 
 * @param motor_temp  nhiet do dong co 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_gmotor_test_mcu2pc_send(mavlink_channel_t chan, uint8_t motor_id, uint8_t motor_status, float motor_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_float(buf, 0, motor_temp);
	_mav_put_uint8_t(buf, 4, motor_id);
	_mav_put_uint8_t(buf, 5, motor_status);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_GMOTOR_TEST_MCU2PC, buf, 6, 245);
#else
	mavlink_mav_gmotor_test_mcu2pc_t packet;
	packet.motor_temp = motor_temp;
	packet.motor_id = motor_id;
	packet.motor_status = motor_status;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_GMOTOR_TEST_MCU2PC, (const char *)&packet, 6, 245);
#endif
}

#endif

// MESSAGE MAV_GMOTOR_TEST_MCU2PC UNPACKING


/**
 * @brief Get field motor_id from mav_gmotor_test_mcu2pc message
 *
 * @return  dia chi dong co 
 */
static inline uint8_t mavlink_msg_mav_gmotor_test_mcu2pc_get_motor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field motor_status from mav_gmotor_test_mcu2pc message
 *
 * @return  trang thay dong co 
 */
static inline uint8_t mavlink_msg_mav_gmotor_test_mcu2pc_get_motor_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field motor_temp from mav_gmotor_test_mcu2pc message
 *
 * @return  nhiet do dong co 
 */
static inline float mavlink_msg_mav_gmotor_test_mcu2pc_get_motor_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a mav_gmotor_test_mcu2pc message into a struct
 *
 * @param msg The message to decode
 * @param mav_gmotor_test_mcu2pc C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_gmotor_test_mcu2pc_decode(const mavlink_message_t* msg, mavlink_mav_gmotor_test_mcu2pc_t* mav_gmotor_test_mcu2pc)
{
#if MAVLINK_NEED_BYTE_SWAP
	mav_gmotor_test_mcu2pc->motor_temp = mavlink_msg_mav_gmotor_test_mcu2pc_get_motor_temp(msg);
	mav_gmotor_test_mcu2pc->motor_id = mavlink_msg_mav_gmotor_test_mcu2pc_get_motor_id(msg);
	mav_gmotor_test_mcu2pc->motor_status = mavlink_msg_mav_gmotor_test_mcu2pc_get_motor_status(msg);
#else
	memcpy(mav_gmotor_test_mcu2pc, _MAV_PAYLOAD(msg), 6);
#endif
}
