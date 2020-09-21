
void Buck1Drive(void);
void SPI_setup(void);
void CurrentandVoltageMeasurements(void);
void Buck1VoltageLoop(void);
void UpdateControlReference(unsigned int POT);
void UpdateDTR(unsigned int POT);
void updateTrigPeriodPercentage(unsigned int perc);
void Buck1SoftStartRoutine(unsigned int POT);

//UART
void UART_setup(void);
void UART_string_out(char buff[]);

void Delay_ms(unsigned int delay);


