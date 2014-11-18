%% functionname: function description
function [power_dB] = rssi2dBm(rssi, lna_gain, band)

	power_dB = 100;

	B24_RSSI_SLOPE_BITSHIFT		= 4;
	B24_RSSI_OFFSET_LNA_LOW		= (-61);
	B24_RSSI_OFFSET_LNA_MED		= (-76);
	B24_RSSI_OFFSET_LNA_HIGH	= (-92);

	B5_RSSI_SLOPE_BITSHIFT		= 7;
	B5_RSSI_SLOPE_MULT			= 7;
	B5_RSSI_OFFSET_LNA_LOW		= (-51);
	B5_RSSI_OFFSET_LNA_MED		= (-61);
	B5_RSSI_OFFSET_LNA_HIGH		= (-84);



	switch lna_gain

	case 1
		power_dB = double(rssi) ./ (2^B24_RSSI_SLOPE_BITSHIFT) + B24_RSSI_OFFSET_LNA_LOW;

	case 2

		power_dB = double(rssi) ./ (2^B24_RSSI_SLOPE_BITSHIFT) + B24_RSSI_OFFSET_LNA_MED;

	case 3

		power_dB = double(rssi) ./ (2^B24_RSSI_SLOPE_BITSHIFT) + B24_RSSI_OFFSET_LNA_HIGH;
	
	end

	power_dB = power_dB;