#include "pin_mux.h"

#include <hal_gpio.h>
#include <hal_platform.h>

bool pin_has_eint(pin_desc_t *pin)
{
	if (NULL == pin)
		return false;
		
	if (pin->pin_info_eint_num >= HAL_EINT_NUMBER_MAX)
		return false;

	return true;
}

bool pin_enable_digital(pin_desc_t *pin)
{
	hal_pinmux_status_t	ret;

	if (!pin_has_gpio(pin))
		return false;

	ret = hal_pinmux_set_function(pin->pin_no, pin->pin_mux_aon_sel_gpio);

	if (ret == 0) {
		pin->pin_current_function = pin->pin_mux_aon_sel_gpio;
		return true;
	}
	else
		return false;
}

bool pin_enable_analog(pin_desc_t *pin)
{
	hal_pinmux_status_t	ret;

	if (!pin_has_adc(pin))
		return false;

	// we need to set the direction to INPUT to enable proper internal resistance.
	hal_gpio_set_direction(pin->pin_no, HAL_GPIO_DIRECTION_INPUT);
	ret = hal_pinmux_set_function(pin->pin_no, pin->pin_mux_aon_sel_adc);

	if (ret == 0) {
		pin->pin_current_function = pin->pin_mux_aon_sel_adc;
		return true;
	}
	else
		return false;
}

bool pin_enable_pwm(pin_desc_t *pin)
{
	hal_pinmux_status_t	ret;

	if (!pin_has_pwm(pin))
		return false;

	// for PWM, we check to prevent excessive set funcion
	if (pin->pin_current_function == pin->pin_mux_aon_sel_pwm) {
		return true;
	}

	ret = hal_pinmux_set_function(pin->pin_no, pin->pin_mux_aon_sel_pwm);

	if (ret == 0) {
		pin->pin_current_function = pin->pin_mux_aon_sel_pwm;
		return true;
	}
	else
		return false;
}

bool pin_enable_uart(pin_desc_t *pin)
{
	hal_pinmux_status_t	ret;

	if (!pin_has_uart(pin))
		return false;

	ret = hal_pinmux_set_function(pin->pin_no, pin->pin_mux_aon_sel_uart);

	if (ret == 0) {
		pin->pin_current_function = pin->pin_mux_aon_sel_uart;
		return true;
	}
	else
		return false;
}

bool pin_enable_i2c(pin_desc_t *pin)
{
	hal_pinmux_status_t	ret;

	if (!pin_has_i2c(pin))
		return false;

	if (pin->pin_current_function != pin->pin_mux_aon_sel_i2c) {

		ret = hal_pinmux_set_function(pin->pin_no, pin->pin_mux_aon_sel_i2c);
		if (ret == 0)
			pin->pin_current_function = pin->pin_mux_aon_sel_i2c;
		else
			return false;
	}

	hal_gpio_set_pupd_register(pin->pin_no, 1, 0, 1);
	hal_gpio_set_driving_current(pin->pin_no, HAL_GPIO_DRIVING_CURRENT_10MA);
    return true;
}

bool pin_enable_spi(pin_desc_t *pin)
{
	hal_pinmux_status_t	ret;

	if (!pin_has_spi(pin))
		return false;

	ret = hal_pinmux_set_function(pin->pin_no, pin->pin_mux_aon_sel_spi);

	if (ret == 0) {
		pin->pin_current_function = pin->pin_mux_aon_sel_spi;
		return true;
	}
	else
		return false;
}

pin_desc_t *get_pin_desc(hal_gpio_pin_t pin_no)
{
	static pin_desc_t mt7933_pins_desc_tab[] = {
		PIN_DESC_0,   /*  HAL_GPIO_0 */
		PIN_DESC_1,   /*  HAL_GPIO_1 */
		PIN_DESC_2,   /*  HAL_GPIO_2 */
		PIN_DESC_3,   /*  HAL_GPIO_3 */
		PIN_DESC_4,   /*  HAL_GPIO_4 */
		PIN_DESC_5,   /*  HAL_GPIO_5 */
		PIN_DESC_6,   /*  HAL_GPIO_6 */
		PIN_DESC_7,   /*  HAL_GPIO_7 */
		PIN_DESC_8,   /*  PIN_DESC_8 */
		PIN_DESC_9,   /*  PIN_DESC_9 */
		PIN_DESC_10,   /*  PIN_DESC_10 */
		PIN_DESC_11,   /*  PIN_DESC_11 */
		PIN_DESC_12,   /*  PIN_DESC_12 */
		PIN_DESC_13,   /*  PIN_DESC_13 */
		PIN_DESC_14,   /*  PIN_DESC_14 */
		PIN_DESC_15,   /*  PIN_DESC_15 */
		PIN_DESC_16,   /*  PIN_DESC_16 */
		PIN_DESC_17,   /*  PIN_DESC_17 */
		PIN_DESC_18,   /*  PIN_DESC_18 */
		PIN_DESC_19,   /*  PIN_DESC_19 */
		PIN_DESC_20,   /*  PIN_DESC_20 */
		PIN_DESC_21,   /*  PIN_DESC_21 */
		PIN_DESC_22,   /*  PIN_DESC_22 */
		PIN_DESC_23,   /*  PIN_DESC_23 */
		PIN_DESC_24,  /* HAL_GPIO_24 */
		PIN_DESC_25,  /* HAL_GPIO_25 */
		PIN_DESC_26,  /* HAL_GPIO_26 */
		PIN_DESC_27,  /* HAL_GPIO_27 */
		PIN_DESC_28,  /* HAL_GPIO_28 */
		PIN_DESC_29,  /* HAL_GPIO_29 */
		PIN_DESC_30,  /* HAL_GPIO_30 */
		PIN_DESC_31,  /* HAL_GPIO_31 */
		PIN_DESC_32,  /* HAL_GPIO_32 */
		PIN_DESC_33,  /* HAL_GPIO_33 */
		PIN_DESC_34,  /* HAL_GPIO_34 */
		PIN_DESC_35,  /* HAL_GPIO_35 */
		PIN_DESC_36,  /* HAL_GPIO_36 */
		PIN_DESC_37,  /* HAL_GPIO_37 */
		PIN_DESC_38,  /* HAL_GPIO_38 */
		PIN_DESC_39,  /* HAL_GPIO_39 */
		PIN_DESC_40,  /* HAL_GPIO_40 */
		PIN_DESC_41,  /* HAL_GPIO_41 */
		PIN_DESC_42,  /* HAL_GPIO_42 */
		PIN_DESC_43,  /* HAL_GPIO_43 */
		PIN_DESC_44,  /* HAL_GPIO_44 */
		PIN_DESC_45,  /* HAL_GPIO_45 */
		PIN_DESC_46,  /* HAL_GPIO_46 */
		PIN_DESC_47,  /* HAL_GPIO_47 */
		PIN_DESC_48,  /* HAL_GPIO_48 */
		PIN_DESC_49,  /* HAL_GPIO_49 */
		PIN_DESC_50,  /* HAL_GPIO_50 */
		PIN_DESC_51,  /* HAL_GPIO_51 */
		PIN_DESC_52   /* HAL_GPIO_52 */
	};
	uint8_t  index = 0;

	//if (pin_no <= HAL_GPIO_7)
	//	return &mt7933_pins_desc_tab[pin_no];

	if (pin_no <= HAL_GPIO_52)
		index = pin_no - (HAL_GPIO_39 - HAL_GPIO_7 - 1);
		return &mt7933_pins_desc_tab[index];

	//if (pin_no <= HAL_GPIO_60)
	//	index = pin_no - (HAL_GPIO_57 - HAL_GPIO_24 - HAL_GPIO_7 - 1);
	//	return &mt7687_pins_desc_tab[index];

	return NULL;
}
