
numTxAntennas = 10;
numRxAntennas = 1;

sampFreq = 40e6;
Ts = 1/sampFreq;
pilotToneFrequency = 1.25e6;

numSamps = 1e3;

time = [0:Ts:numSamps*Ts].';
symbols =  exp (time*j*2*pi*pilotToneFrequency);


for pktIndex  = 1:10000

	H =  1./sqrt(2) * (randn(numRxAntennas,numTxAntennas) ...
		               + j*randn(numRxAntennas,numTxAntennas)); 

	% H =  1./sqrt(2) * (ones(numRxAntennas,numTxAntennas) ...
	%                + j*ones(numRxAntennas,numTxAntennas)); 

	blindPrecoder = ones(numTxAntennas,1)./sqrt(numTxAntennas);

	% mfPrecoder = H' ./ sqrt(sum(abs(H').^2));

	txVector = (blindPrecoder * symbols .') .';
 
	% chanPower(pktIndex) = abs(sum(H).^2);

	rx_symbols = (H * txVector .') .';

	rxPower(pktIndex) = mean(abs(rx_symbols).^2);

end

% meanChanPower = mean(chanPower)

meanRxPower = mean(rxPower)

meanRxPower_dB = pow2db(mean(rxPower))