%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% zeroForce_v1.m
% By Evan Everett, leveraging example code form Mango Comm. 
% In this example, we estimate the MIMO channel matrix between two 
% communication nodes equipped with mutliple antennas. Narrowband 
% communication is assumed. The number of transmit and receive antennas
% can be specified arbitrarily at the beggining of the script. But if 
% running in harwdare mode, must match your hardware configuration. 
%
% The training pilots are constructed orthogonal in time: Each transmit
% antenna is given its own slot in which to transmit a pilot. The receiver
% then estimates the channels from each transmit antenna to each of its 
% receive antennas using the orthogonal pilots. 
%
% After the channel is estimated, a packet is transmitted which
% is zero-forced to one of the received antennas but not to the other antennas. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; 
clc; 
close all;

cf = 0;

%%%%%%%%%%%%%%%%%  User Parameters  %%%%%%%%%%%%%%
USE_AGC = true;
USE_WARPLAB_TXRX = false;

%Use sane defaults for hardware-dependent params in sim-only version
maxTxLength = 32768;
sampFreq = 40e6;
TXRX_DELAY = 0;

NUMNODES = 2; %must be set to 2 for now

% choose how many transmit and receive antennas
% you wish to use. Can be arbitrarty in simulation, 
% when using with WARP boards, must be less than 4. 
numTxAntennas = 2;
numRxAntennas = 1;

% There is ringing in the recived sigal 
% that lasts nearly 4e3 samples. 
% We will just transmit a sinusoid in that window. 
numSamplesLetSettle = 4e3;

% choose the length of the pilot signal. Longer 
% pilots lead to better estimation accuracy but more overhead.
pilotLength = 1024; % length of per-antenna pilot symbol in samples

% Pilot tone frequency. 
% The pilot will be a tone at a given frequency. %
% the frequency should be the center frequency around which communication will occur. 
pilotToneFrequency = 1.25e6;

% Choose the length of the guard interval between pilots from each antenna. 
% This may not be necessary, but it does greatly aid in the visualization of 
% the orthogonal pilots
guardIntervalLength = 256;

% Payload tone frequency
% The payload will just be a tone, pick its frequency. 
% Can be the same as as the pilotToneFrequency
payloadToneFreq = 1.25e6;

NUM_SHORT_SYMS_REP = 30;

%%%%%%%%% Simulation Parameters %%%%%%%%%%%%%%%%
% Choose a SNR for the sim-only version. 
snr_dB = 30;


if USE_WARPLAB_TXRX
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Set up the WARPLab experiment
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	USE_AGC = true;

	%External trigger mode requires a connection from TRIGOUT_D0 on node 0
	% to TRIGIN_D3 on node 1 (see http://warpproject.org/w/WARPLab/Examples for details)
	USE_EXTERNAL_TRIGGER = true;
	TXRX_DELAY = 45; % not exact

	%Create a vector of node objects
	% Note: comment out wl_initNodes after running once to keep constant
	%  phase offsets among nodes sharing an RF reference clock.
	%  wl_initNodes() executes a reset of the MAX2829 transceivers, which
	%  forces a re-tune of the PLL, which changes the inter-node phases.
	nodes = wl_initNodes(NUMNODES);

	%Create a UDP broadcast trigger and primary node to be ready for it
	eth_trig = wl_trigger_eth_udp_broadcast;

	%TODO: Only trigger the transmitting node with an Ethernet trigger
	nodes.wl_triggerManagerCmd('add_ethernet_trigger',[eth_trig]);

	%Read Trigger IDs into workspace
	[T_IN_ETH,T_IN_ENERGY,T_IN_AGCDONE,T_IN_REG,T_IN_D0,T_IN_D1,T_IN_D2,T_IN_D3] =  wl_getTriggerInputIDs(nodes(1));
	[T_OUT_BASEBAND, T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3] = wl_getTriggerOutputIDs(nodes(1));

	%For the transmit node, we will allow Ethernet to trigger the buffer
	%baseband, the AGC, and debug0 (which is mapped to pin 8 on the debug
	%header)
	nodes(1).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_D0],[T_IN_ETH,T_IN_REG]);

	if(USE_EXTERNAL_TRIGGER)
	    %For the receive node, we will allow debug3 (mapped to pin 15 on the
	    %debug header) to trigger the buffer baseband, and the AGC
	    nodes(2).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC],[T_IN_D3]);
	else
	    nodes(2).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC],[T_IN_ETH,T_IN_REG]);
	end

	%For the receive node, we enable the debounce circuity on the debug 3 input
	%to deal with the fact that the signal may be noisy.
	nodes(2).wl_triggerManagerCmd('input_config_debounce_mode',[T_IN_D3],'enable'); 

	%Since the debounce circuitry is enabled, there will be a delay at the
	%receiver node for its input trigger. To better align the transmitter and
	%receiver, we can artifically delay the transmitters trigger outputs that
	%drive the buffer baseband and the AGC.
	nodes(1).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[50]); %50ns delay

	%Get IDs for the interfaces on the boards. Since this example assumes each
	%board has the same interface capabilities, we only need to get the IDs
	%from one of the boards
	[RFA,RFB] = wl_getInterfaceIDs(nodes(1));

	%Set up the interface for the experiment
	wl_interfaceCmd(nodes,'RF_ALL','tx_gains',3,30);
	wl_interfaceCmd(nodes,'RF_ALL','channel',2.4,11);

	if(USE_AGC)
	    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','automatic');
	    wl_basebandCmd(nodes,'agc_target',-10);
	    wl_basebandCmd(nodes,'agc_trig_delay', 500);
	else
	    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','manual');
	    RxGainRF = 1; %Rx RF Gain in [1:3]
	    RxGainBB = 17; %Rx Baseband Gain in [0:31]
	    wl_interfaceCmd(nodes,'RF_ALL','rx_gains',RxGainRF,RxGainBB);
	end


	%We'll use the transmitter's I/Q buffer size to determine how long our
	%transmission can be
	maxTxLength = nodes(2).baseband.txIQLen;

end % if USE_WARPLAB_TXRX

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Signal processing to generate transmit signal for channel estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ts = 1/sampFreq;

% First generate the preamble for AGC. The preamble corresponds to the
% short symbols from the 802.11a PHY standard
shortSymbol_freq = [0 0 0 0 0 0 0 0 1+i 0 0 0 -1+i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 0 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 -1+i 0 0 0 1+i 0 0 0 0 0 0 0].';
shortSymbol_freq = [zeros(32,1);shortSymbol_freq;zeros(32,1)];
shortSymbol_time = ifft(fftshift(shortSymbol_freq));
shortSymbol_time = (shortSymbol_time(1:32).')./max(abs(shortSymbol_time));
shortsyms_rep = repmat(shortSymbol_time,1,NUM_SHORT_SYMS_REP);

timeForSettlinSymbols = [0:Ts:numSamplesLetSettle*Ts].';
symbolsToLetReceiverSettle = .6 * exp (timeForSettlinSymbols*j*2*pi*pilotToneFrequency);

% preamble = shortsyms_rep;
% preamble = preamble(:);

preamble = [shortsyms_rep(:) ; symbolsToLetReceiverSettle(:)];

% Create time vector(Sample Frequency is Ts (Hz))
t = [0:Ts:((maxTxLength-length(preamble)-1))*Ts].'; 




% create the pilot signal
signalPilot = 0.6 * exp (t(1:pilotLength)*j*2*pi*pilotToneFrequency);

% this will be the zero vectore transmitted when another antennas is sending its training
zeroPilot = zeros(size(signalPilot));

% guard interval to place between different
% fields within the packet. Mostly to eas visualization. 
gaurdInterval = zeros(guardIntervalLength,1);

pktLength = length(preamble) + numTxAntennas*pilotLength ...
            + (numTxAntennas + 1) * guardIntervalLength;

trainSignal = zeros(pktLength ,numTxAntennas);

pilotStencil = repmat([gaurdInterval; signalPilot],numTxAntennas,1);

txPilotStartIndices = zeros(1,numTxAntennas);

% orthogonal training symbols for each of the antennas
for txAntenna = 1:numTxAntennas;
	% Indicator vector for which antenna we are crafting pilot for. 
	antennaIndicator = (txAntenna == 1:numTxAntennas);

	% Mask that will zero out the training pilot for all but slot assigned to active antenna
	orthogonalTrainMask = repmat(antennaIndicator, guardIntervalLength+pilotLength,1);
	orthogonalTrainMask = reshape(orthogonalTrainMask, numTxAntennas*(guardIntervalLength+pilotLength), 1);

	% Create the train signal
	trainSignal(:,txAntenna) = [preamble; orthogonalTrainMask .* pilotStencil; gaurdInterval; ];

	txPilotStartIndices(txAntenna) = length(preamble) + (txAntenna-1) * pilotLength ...
											+ txAntenna * guardIntervalLength + TXRX_DELAY;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The channel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if USE_WARPLAB_TXRX == false;

	% generate a random channel matrix
	H = rand(numRxAntennas,numTxAntennas) + j*rand(numRxAntennas,numTxAntennas); 

	% generate an AWGN noise vector
	rx_noise = (1/db2mag(snr_dB)) *  complex(randn(numRxAntennas,pktLength), ...
	                                         randn(numRxAntennas,pktLength)) .';

	% apply the channel model. 
	rx_IQ = (H * trainSignal .') .' + rx_noise;
	rx_RSSI = mag2db(abs(rx_IQ));

else
	node_tx = nodes(1);
	node_rx = nodes(2);
	% RF_TX = RFA;
	% RF_RX = RFA;


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Transmit and receive signal using WARPLab
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	txLength = length(trainSignal);

	%Set up the baseband for the experiment
	wl_basebandCmd(nodes,'tx_delay',0);
	wl_basebandCmd(nodes,'tx_length',txLength);

	wl_basebandCmd(node_tx,[RFA, RFB], 'write_IQ', trainSignal);
	wl_interfaceCmd(node_tx, RFA+RFB,'tx_en');
	wl_interfaceCmd(node_rx, RFA,'rx_en');

	wl_basebandCmd(node_tx, RFA+RFB,'tx_buff_en');
	wl_basebandCmd(node_rx, RFA,'rx_buff_en');

	eth_trig.send();
	rx_IQ = wl_basebandCmd(node_rx,[RFA],'read_IQ', 0, txLength);
	rx_RSSI = mag2db(abs(rx_IQ));  % for now. We'll fix this later.   

	wl_basebandCmd(nodes,'RF_ALL','tx_rx_buff_dis');
	wl_interfaceCmd(nodes,'RF_ALL','tx_rx_dis');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Signal processing to estimate channel from received signal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Slice out the pilots for estimating each element of the channel matrix ----
% Create a matrix with numTxAntennas*numRxAntennas columns and pilotLength rows.
% Each column vector is the pilots for estimating one of the elements of the 
% channel matrix, the the rows correspond to the elements of the channel matrix


% The below two blocks are identical functionally but the
% first block is more efficient. The second block is more readable
% and hence we leave it commented. 
rx_pilots = zeros(pilotLength, numTxAntennas*numRxAntennas);
for txAntenna = 1:numTxAntennas

	pilots = rx_IQ(txPilotStartIndices(txAntenna)+1 ...
		           : txPilotStartIndices(txAntenna)+pilotLength,:);

	rx_pilots(:,(txAntenna-1)*numRxAntennas+1 : ...
		         txAntenna*numRxAntennas) = pilots;
						    
end

% rx_pilots = zeros(pilotLength,0);
% for txAntenna = 1:numTxAntennas
% 	rx_pilots = [rx_pilots, rx_IQ(txPilotStartIndices(txAntenna)+1 : ...
% 		                          txPilotStartIndices(txAntenna)+pilotLength,:)];
% end



% just divide each by the pilots
H_est = reshape(mean(rx_pilots ./ repmat(signalPilot,1,numTxAntennas*numRxAntennas)),numRxAntennas,numTxAntennas);

if USE_WARPLAB_TXRX == false;

	% compute the average channel estimatio error
	channelEstimationError = pow2db (mean(reshape(abs(H - H_est).^2 ./ abs(H).^2, ...
		numTxAntennas*numRxAntennas,1) ));
	fprintf('Channel estimation error is %.2f dB\n', channelEstimationError)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize results for channel estimation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cf = cf+1;
figure(cf);clf;
colorOrder = get(gca, 'ColorOrder');
for rxAnt = 1:numRxAntennas
	ax(rxAnt,1) = subplot(numRxAntennas+1,2,2*(rxAnt-1)+1);
	plot(0:(length(rx_IQ)-1),real(rx_IQ(:,rxAnt)), 'color', colorOrder(rxAnt,:))
	xlabel('Sample Index')
	title(strcat('Antenna #', num2str(rxAnt), ' Received I'))
	axis tight;
	ylim([-1.5,1.5]);

	ax(rxAnt,2) = subplot(numRxAntennas+1,2,2*(rxAnt-1)+2);
	plot(0:(length(rx_IQ)-1),imag(rx_IQ(:,rxAnt)), 'color', colorOrder(rxAnt,:))
	xlabel('Sample Index')
	title(strcat('Antenna #', num2str(rxAnt), ' Received Q'))
	axis tight
	ylim([-1.5,1.5])
end

linkaxes(ax,'x')
subplot(numRxAntennas+1,1,numRxAntennas+1)
hold on
for rxAnt = 1:numRxAntennas
	plot(0:(length(rx_RSSI)-1),rx_RSSI(:,rxAnt), 'color', colorOrder(rxAnt,:))
end
hold off
%legend('RFA','RFB','location','southeast')
axis tight
xlabel('Sample Index')
title('Received RSSI')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create the transit signal, 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

payloadLength = maxTxLength - length(trainSignal);

txLength = maxTxLength;

% create time index for the payload
t = [0:Ts:((payloadLength-1))*Ts].'; 

payload = .6*exp(t*j*2*pi*payloadToneFreq); %5 MHz sinusoid as our "payload" for RFA

% modulate with a a quare wave at mush lower frequency
%payload = (.5 * (1 + square(t*payloadToneFreq/10))) .* payload; 

% maximal ratio combining
%precoder = H_est';

%precoder = [1 ; 1]./sum(H_est)

% null combining
precoder = null(H_est);

payload_precoded = (precoder * payload .') .';

max_mag = max(max(abs(payload_precoded)));
if max(abs(payload_precoded)) > 1
	payload_precoded = 0.9 * payload_precoded./ max_mag;
	payload = 0.9 * payload ./ max_mag;
	print('payload scaled to avoid clippings')
end

txData = [trainSignal; payload_precoded];

figure; plot(real(txData));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The channel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if USE_WARPLAB_TXRX == false; 

	% generate an AWGN noise vector
	rx_noise = (1/db2mag(snr_dB)) *  complex(randn(numRxAntennas,txLength), ...
	                                         randn(numRxAntennas,txLength)) .';

	% apply the channel model. 
	rx_IQ = (H * txData .') .' + rx_noise;
	rx_RSSI = mag2db(abs(rx_IQ));

else

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Transmit and receive signal using WARPLab
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	node_tx = nodes(1);
	node_rx = nodes(2);
	% RF_TX = RFA;
	% RF_RX = RFA;

	%Set up the baseband for the experiment
	wl_basebandCmd(nodes,'tx_delay',0);
	wl_basebandCmd(nodes,'tx_length',txLength);

	wl_basebandCmd(node_tx,[RFA, RFB], 'write_IQ', txData);
	wl_interfaceCmd(node_tx, RFA+RFB,'tx_en');
	wl_interfaceCmd(node_rx, RFA,'rx_en');

	wl_basebandCmd(node_tx, RFA+RFB,'tx_buff_en');
	wl_basebandCmd(node_rx, RFA,'rx_buff_en');

	eth_trig.send();
	rx_IQ = wl_basebandCmd(node_rx,[RFA],'read_IQ', 0, txLength);
	rx_RSSI = mag2db(abs(rx_IQ));  % for now. We'll fix this later.   

	wl_basebandCmd(nodes,'RF_ALL','tx_rx_buff_dis');
	wl_interfaceCmd(nodes,'RF_ALL','tx_rx_dis');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize results for channel estimation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cf = cf+1;
figure(cf);clf;
colorOrder = get(gca, 'ColorOrder');
for rxAnt = 1:numRxAntennas
	ax(rxAnt,1) = subplot(numRxAntennas+1,2,2*(rxAnt-1)+1);
	plot(0:(length(rx_IQ)-1),real(rx_IQ(:,rxAnt)), 'color', colorOrder(rxAnt,:))
	xlabel('Sample Index')
	title(strcat('Antenna #', num2str(rxAnt), ' Received I'))
	axis tight;
	ylim([-1.5,1.5]);

	ax(rxAnt,2) = subplot(numRxAntennas+1,2,2*(rxAnt-1)+2);
	plot(0:(length(rx_IQ)-1),imag(rx_IQ(:,rxAnt)), 'color', colorOrder(rxAnt,:))
	xlabel('Sample Index')
	title(strcat('Antenna #', num2str(rxAnt), ' Received Q'))
	axis tight
	ylim([-1.5,1.5])
end

linkaxes(ax,'x')
subplot(numRxAntennas+1,1,numRxAntennas+1)
hold on
for rxAnt = 1:numRxAntennas
	plot(0:(length(rx_RSSI)-1),rx_RSSI(:,rxAnt), 'color', colorOrder(rxAnt,:))
end
hold off
%legend('RFA','RFB','location','southeast')
axis tight
xlabel('Sample Index')
title('Received RSSI')

% get the received pilots
rx_pilots = zeros(pilotLength, numTxAntennas*numRxAntennas);
for txAntenna = 1:numTxAntennas

	pilots = rx_IQ(txPilotStartIndices(txAntenna)+1 ...
		           : txPilotStartIndices(txAntenna)+pilotLength,:);

	rx_pilots(:,(txAntenna-1)*numRxAntennas+1 : ...
		         txAntenna*numRxAntennas) = pilots;
						    
end

% just divide each by the pilots
H_eq = reshape(mean(rx_pilots ./ repmat(signalPilot,1,numTxAntennas*numRxAntennas)),numRxAntennas,numTxAntennas);

if USE_WARPLAB_TXRX == false;

	% compute the average channel estimatio error
	channelEstimationError = pow2db (mean(reshape(abs(H - H_eq).^2 ./ abs(H).^2, ...
		numTxAntennas*numRxAntennas,1) ));
	fprintf('Channel estimation error is %.2f dB\n', channelEstimationError)
end



payloadStart = length(trainSignal) + TXRX_DELAY + 1;

rx_payload = rx_IQ(payloadStart:end);

figure; plot(real(txData))

figure; 
plot(real(payload(1:300)),'b'); hold on
plot(real(rx_payload(1:300)),'r');
ylim([-1,1])
hold off

%rx_payload_eq = rx_payload ./ sum(H_eq);
rx_payload_eq = rx_payload ;

figure; 
plot(real(payload(1:1e3)),'b'); hold on
plot(real(rx_payload_eq(1:1e3)),'r');
ylim([-1,1])
hold off

figure; 
plot(real(payload(1:length(rx_payload_eq))),'b'); hold on
plot(real(rx_payload_eq),'r');
ylim([-1,1])
hold off

% there will be some error at first and last, so igore that
% n_samp_ignore = 0;
% precodingError = pow2db(mean(abs(payload(n_samp_ignore+1:length(rx_payload_eq)-n_samp_ignore) - rx_payload_eq(n_samp_ignore+1:end-n_samp_ignore)).^2));

rssiPreambleStart = length(symbolsToLetReceiverSettle) + 1;

rssiPreambleStop = rssiPreambleStart + numSamplesLetSettle;

rx_RSSI_Samples = rx_IQ(rssiPreambleStart + TXRX_DELAY : rssiPreambleStop + TXRX_DELAY);


powerUnsupressed = pow2db(mean(abs(rx_RSSI_Samples).^2));

powerSupressed = pow2db(mean(abs(rx_payload).^2));

suppression = powerUnsupressed - powerSupressed

