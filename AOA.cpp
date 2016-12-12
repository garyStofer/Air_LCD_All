#include "AOA.h"

#ifdef WITH_AOA


float V_AOA;

void aoaRead()
{
	short adc_val;

	adc_val = analogRead(AOA_ADC);
	V_AOA = adc_val * AOA_ADC_BW;
	V_AOA = V_AOA - AOA_0;
	V_AOA = V_AOA / ((AOA_90 - AOA_0)/90);

}
#endif
