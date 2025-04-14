#ifndef _PERIPHERAL_SENSECAP_H_
#define _PERIPHERAL_SENSECAP_H_

#ifdef __cplusplus
#include <cstdint>
#endif

/*!
 * @brief Get temperature value
 * 
 * @return temperature value, in celsius x 10
 */
int16_t t1000e_ntc_sample(void);

/*!
 * @brief Get light value
 * 
 * @return light value, in percentage
 */
int16_t t1000e_lux_sample(void);

/*!
 * @brief Get battery value
 * 
 * @return battery value, in percentage
 */
int16_t t1000e_bat_sample(void);

#endif /*_PERIPHERAL_SENSECAP_H_ */
