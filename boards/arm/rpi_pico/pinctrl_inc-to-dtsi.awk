#!/bin/env awk -f
# Read pinctrl file for rp2040 pico and emit equivalent dts pin statements.
# For multipin interfaces, order matters!
#
# e.g:  awk <../include/dt-bindings/pinctrl/rpi-pico-rp2040-pinctrl.h -f f.awk

BEGIN			{}

# Expected input, UART:
#* #define UART0_TX_P0 RP2040_PINMUX(0, PINCTRL_GPIO_FUNC_UART)
#* #define UART0_RX_P1 RP2040_PINMUX(1, PINCTRL_GPIO_FUNC_UART)
#* #define UART0_CTS_P2 RP2040_PINMUX(2, PINCTRL_GPIO_FUNC_UART)
#* #define UART0_RTS_P3 RP2040_PINMUX(3, PINCTRL_GPIO_FUNC_UART)

/define UART[0-9]_TX.*_PINMUX/	{ inst=substr($2, 5, 1); pin=tolower(substr($2, 10)); uart_tx=$2; }
/define UART[0-9]_RX.*_PINMUX/	{ uart_rx=$2; }
/define UART[0-9]_CTS.*_PINMUX/	{ uart_cts=$2; }
/define UART[0-9]_RTS.*_PINMUX/	{ uart_rts=$2;
                                  printf("&uart%s_%s: {\n\tgroup1 { pinmux = <%s>; pinmux = <%s>; }\n", inst, pin, uart_tx, uart_cts);
				  printf("\tgroup2 { pinmux = <%s>; pinmux = <%s>; input-enable; }\n}\n", uart_rx, uart_rts);
				}


# Expected input, SPI:
#* #define SPI0_RX_P0 RP2040_PINMUX(0, PINCTRL_GPIO_FUNC_SPI)
#* #define SPI0_CSN_P1 RP2040_PINMUX(1, PINCTRL_GPIO_FUNC_SPI)
#* #define SPI0_SCK_P2 RP2040_PINMUX(2, PINCTRL_GPIO_FUNC_SPI)
#* #define SPI0_TX_P3 RP2040_PINMUX(3, PINCTRL_GPIO_FUNC_SPI)

/define SPI[0-9]_RX.*_PINMUX/	{ inst=substr($2, 4, 1); pin=tolower(substr($2, 9)); spi_rx=$2; }
/define SPI[0-9]_CSN.*_PINMUX/	{ spi_csn=$2; }
/define SPI[0-9]_SCK.*_PINMUX/	{ spi_sck=$2; }
/define SPI[0-9]_TX.*_PINMUX/	{ spi_tx=$2;
                                  printf("&spi%s_%s: {\n\tgroup1 { pinmux = <%s>; }\n", inst, pin, spi_tx);
				  printf("\tgroup2 { pinmux = <%s>; pinmux = <%s>; pinmux = <%s>; input-enable; }\n}\n", spi_rx, spi_csn, spi_sck);
				}


# Expected input, I2C:
#* #define I2C0_SDA_P0 RP2040_PINMUX(0, PINCTRL_GPIO_FUNC_I2C)
#* #define I2C0_SCL_P1 RP2040_PINMUX(1, PINCTRL_GPIO_FUNC_I2C)

/define I2C[0-9]_SDA.*_PINMUX/	{ inst=substr($2, 4, 1); pin=tolower(substr($2, 10)); i2c_sda=$2; }
/define I2C[0-9]_SCL.*_PINMUX/	{ i2c_scl=$2;
                                  printf("&i2c%s_%s: {\n\tgroup1 { pinmux = <%s>; input-enable; }\n", inst, pin, i2c_sda);
				  printf("\tgroup2 { pinmux = <%s>; input-enable; }\n}\n", i2c_scl);
				}


# Expected input, GPIO/SIO:
#* #define SIO_P0 RP2040_PINMUX(0, PINCTRL_GPIO_FUNC_SIO)

/define SIO.*_PINMUX/		{ printf("&%s: { pinmux = <%s>; }\n", tolower($2), $2); }


# Expected input, PWM:
#* #define PWM0_A_P0 RP2040_PINMUX(0, PINCTRL_GPIO_FUNC_PWM)
#* #define PWM0_B_P1 RP2040_PINMUX(1, PINCTRL_GPIO_FUNC_PWM)

/define PWM[0-9]_A.*_PINMUX/	{ inst=substr($2, 4, 1); pin=tolower(substr($2, 8)); pwm_a=$2; 
                                  printf("&pwm%sa_%s: { pinmux = <%s>; }\n", inst, pin, pwm_a);
				}
/define PWM[0-9]_B.*_PINMUX/	{ inst=substr($2, 4, 1); pin=tolower(substr($2, 8)); pwm_a=$2; pwm_b=$2;
                                  printf("&pwm%sb_%s: { pinmux = <%s>; }\n", inst, pin, pwm_b);
				}


