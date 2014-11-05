%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% In this experiment, we will have an array of M_T transmit antennas.
% M_R receive antennas, K Users. The simplest, first-cut case will be
% Two transmit antennas (M_T = 2) One user, K=1, and one receive antenna,
% M_R = 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; 
clc; 
close all;
cf = 0;


%%%%%%%%%%%%%%%%%  User Parameters  %%%%%%%%%%%%%%
USE_AGC = true;
USE_WARPLAB_TXRX = false;

numPkts = 100;


%Use sane defaults for hardware-dependent params in sim-only version
maxTxLength = 32768;
sampFreq = 40e6;
Ts = 1/sampFreq;
TXRX_DELAY = 0;

NUMNODES = 2; %must be set to 2 for now

% choose how many transmit and receive antennas
% you wish to use. Can be arbitrarty in simulation, 
% when using with WARP boards, must be less than 4. 
numTxAntennas = 2;
numRxAntennas = 1;

numUsers = 1;

% There is ringing in the recived sigal 
% that lasts nearly 4e3 samples. a
% We will just transmit a sinusoid in that window. 
numSamplesLetSettle = 4e3;

numSampsForUncodedEstimation = 1000;

% choose the length of the pilot signal. Longer 
% pilots lead to better estimation accuracy but more overhead.
pilotLength = 128; % length of per-antenna pilot symbol in samples

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
MODEL_NOISE = true;
MODEL_FADING = true;
% MODEL_CFO = false;
% MODEL_DELAY = false;

signalAmplitude = 1.0;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the WARPLab experiment
% -- We only need to do this once at beginning, not for each packet
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if USE_WARPLAB_TXRX
	
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

	TX_NUM_SAMPS = nodes(1).baseband.txIQLen;
    % SAMP_FREQ = wl_basebandCmd(nodes(1),'tx_buff_clk_freq'); 


	%We'll use the transmitter's I/Q buffer size to determine how long our
	%transmission can be
	maxTxLength = nodes(2).baseband.txIQLen;
	sampFreq = wl_basebandCmd(nodes(2),'tx_buff_clk_freq'); 
	Ts = 1/sampFreq;

	node_tx = nodes(1);
	node_rx = nodes(2);
	% RF_TX = RFA;
	% RF_RX = RFA;

end % if USE_WARPLAB_TXRX



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Phase 1: Training 
% 
% 1.1 Base station constructs training packet
% 1.2 Base station transmits training packet
% 1.3 User(s) and base station process received signal to estimat channels
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1.1 Signal processing to generate transmit signal for channel estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% First generate the preamble for AGC. The preamble corresponds to the
% short symbols from the 802.11a PHY standard
shortSymbol_freq = [0 0 0 0 0 0 0 0 1+i 0 0 0 -1+i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 0 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 -1+i 0 0 0 1+i 0 0 0 0 0 0 0].';
shortSymbol_freq = [zeros(32,1);shortSymbol_freq;zeros(32,1)];
shortSymbol_time = ifft(fftshift(shortSymbol_freq));
shortSymbol_time = (shortSymbol_time(1:32).')./max(abs(shortSymbol_time));
shortsyms_rep = repmat(shortSymbol_time,1,NUM_SHORT_SYMS_REP);

timeForSettlinSymbols = [0:Ts:numSamplesLetSettle*Ts].';
symbolsToLetReceiverSettle = signalAmplitude * exp (timeForSettlinSymbols*j*2*pi*pilotToneFrequency);

% preamble = shortsyms_rep;
% preamble = preamble(:);

preamble = [shortsyms_rep(:) ; symbolsToLetReceiverSettle(:)] ./sqrt(numTxAntennas);

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

plotWaveforms = true ;
selfIntSuppression = zeros(1,numPkts);
MF_beamformGain = zeros(1,numPkts);

for pktIndx = 1:numPkts

	fprintf('\n\nPacket # %d:\n', pktIndx)

	if pktIndx > 1
		plotWaveforms = false ;
	end


	% This script transmit the training packet and has all the users estimate their channel
	% responses. % It is  global script. The only value that will be set (hopefully)
	% is H the simulated channel and H_est the estimated channel matrix. 
	soundChannels


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%
	% Phase 2: Transmission of data packet from base station  
	% 
	% 2.1 Base station constructs data packet
	% 2.2 Base station transmits pacet to user(s)
	% 2.3 User(s) and base station process received signal and compute performance
	%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.1 Create the transit signal, 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	payloadLength = maxTxLength - length(trainSignal);

	txLength = maxTxLength;

	% create time index for the payload
	t = [0:Ts:((payloadLength-1))*Ts].'; 

	payload = signalAmplitude * exp(t*j*2*pi*payloadToneFreq); %5 MHz sinusoid as our "payload" for RFA

	% null combining
	channelNullspace = null(H_est);


	if isempty(channelNullspace)
		precoder = ones(numTxAntennas,1)./sqrt(numTxAntennas);
		warning('Cannot zero force')
	else
		precoder = channelNullspace(:,1);
	end
	% precoder = channelNullspace(:,1) ...
	% 	./ sqrt(sum(abs(channelNullspace(:,1)).^2))

	if abs(sum(abs(precoder).^2) - 1) > 1e-6 
		error('precoder does not have unity power')
	end

	precoderPower = pow2db(sum(abs(precoder).^2));

	payload_precoded = (precoder * payload .') .';

	max_mag = max(max(abs(payload_precoded)));
	if max(abs(payload_precoded)) > 1
		payload_precoded = 0.9 * payload_precoded./ max_mag;
		payload = 0.9 * payload ./ max_mag;
		print('payload scaled to avoid clippings')
	end

	txData = [trainSignal; payload_precoded];


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.2 base station transmits its packet and users receive
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	transmitData


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.3 Users and base station process their received signals.
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Visualize results for channel estimation 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.3.1 Estimate Channels for equalization
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


	%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.3.2 Payload processing
	%%%%%%%%%%%%%%%%%%%%%%%%%%

	payloadStart = length(trainSignal) + TXRX_DELAY + 1;

	rx_payload = rx_IQ(payloadStart:end);

	%rx_payload_eq = rx_payload ./ sum(H_eq);
	rx_payload_eq = rx_payload ;

	% there will be some error at first and last, so igore that
	% n_samp_ignore = 0;
	% precodingError = pow2db(mean(abs(payload(n_samp_ignore+1:length(rx_payload_eq)-n_samp_ignore) - rx_payload_eq(n_samp_ignore+1:end-n_samp_ignore)).^2));



	rssiPreambleStop = length(preamble);

	rssiPreambleStart = rssiPreambleStop - numSampsForUncodedEstimation;

	rx_RSSI_Samples = rx_IQ(rssiPreambleStart + TXRX_DELAY : rssiPreambleStop + TXRX_DELAY);


	powerUnsupressed = pow2db(mean(abs(rx_RSSI_Samples).^2));

	powerSupressed = pow2db(mean(abs(rx_payload).^2));

	if USE_WARPLAB_TXRX == false
		noiseFloor = pow2db(mean(abs(rx_noise(payloadStart:end)).^2));
	end

	nonCoherenctPower(pktIndx) = powerUnsupressed;

	suppression = powerUnsupressed - powerSupressed;

	selfIntSuppression(pktIndx) = suppression;

	if plotWaveforms	
		figure('Name', 'Payload Rx vs Tx Zoomed'); 
		plot(real(payload(1:300)),'b'); hold on
		plot(real(rx_payload(1:300)),'r');
		ylim([-1,1])
		hold off
		% figure('Name', 'Payload Rx vs Tx'); 
		% plot(real(payload(1:1e3)),'b'); hold on
		% plot(real(rx_payload(1:1e3)),'r');
		% ylim([-1,1])
		% hold off

		figure('Name', 'Payload Rx vs Tx Equalized'); 
		plot(real(payload(1:length(rx_payload_eq))),'b'); hold on
		plot(real(rx_payload_eq),'r');
		ylim([-1,1])
		hold off
	end % if plot waveforms

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Phase 3 matched filter transmission
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Phase 3.1 Now reestimate the channels
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	soundChannels


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 3.2 Create the transit signal, 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	payloadLength = maxTxLength - length(trainSignal);

	txLength = maxTxLength;

	% create time index for the payload
	t = [0:Ts:((payloadLength-1))*Ts].'; 

	payload = signalAmplitude*exp(t*j*2*pi*payloadToneFreq); %5 MHz sinusoid as our "payload" for RFA

	% modulate with a a quare wave at mush lower frequency
	% payload = (.5 * (1 + square(t*payloadToneFreq/10))) .* payload; 

	% conjugate beamforming
	precoder = H_est' ./ sqrt(sum(abs(H_est).^2)) ;

	% Blind precoder 
	%precoder =[1; 1]./sqrt(numTxAntennas);

	precoderPower = pow2db(sum(abs(precoder).^2));

	% null combining
	%precoder = null(H_est);

	payload_precoded = (precoder * payload .') .';

	max_mag = max(max(abs(payload_precoded)));
	if max(abs(payload_precoded)) > 1
		payload_precoded = 0.9 * payload_precoded./ max_mag;
		payload = 0.9 * payload ./ max_mag;
		print('payload scaled to avoid clippings')
	end

	txData = [trainSignal; payload_precoded];


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 3.3 Transmit the data
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	transmitData

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 3.4.1 Estimate Channels for equalization
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


	%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.3.2 Payload processing
	%%%%%%%%%%%%%%%%%%%%%%%%%%

	rx_payload = rx_IQ(payloadStart:end);

	%rx_payload_eq = rx_payload ./ sum(H_eq);
	rx_payload_eq = rx_payload ;

	% there will be some error at first and last, so igore that
	% n_samp_ignore = 0;
	% precodingError = pow2db(mean(abs(payload(n_samp_ignore+1:length(rx_payload_eq)-n_samp_ignore) - rx_payload_eq(n_samp_ignore+1:end-n_samp_ignore)).^2));


	rx_RSSI_Samples = rx_IQ(rssiPreambleStart + TXRX_DELAY : rssiPreambleStop + TXRX_DELAY);


	powerUnprecoded = pow2db(mean(abs(rx_RSSI_Samples).^2));

	powerPrecoded = pow2db(mean(abs(rx_payload).^2));

	beamformingGain = powerPrecoded - powerUnprecoded;

	MF_beamformGain(pktIndx) = beamformingGain;

	if plotWaveforms	
		figure('Name', 'MF Payload Rx vs Tx Zoomed'); 
		plot(real(payload(1:300)),'b'); hold on
		plot(real(rx_payload(1:300)),'r');
		ylim([-1,1])
		hold off
		% figure('Name', 'Payload Rx vs Tx'); 
		% plot(real(payload(1:1e3)),'b'); hold on
		% plot(real(rx_payload(1:1e3)),'r');
		% ylim([-1,1])
		% hold off

		figure('Name', 'MF Payload Rx vs Tx Equalized'); 
		plot(real(payload(1:length(rx_payload_eq))),'b'); hold on
		plot(real(rx_payload_eq),'r');
		ylim([-1,1])
		hold off
	end % plot waveforms

end

meanSuppression = pow2db(mean(db2pow(selfIntSuppression)));

fprintf('\nMean self-interference Suppression: %.1f dB\n', meanSuppression)

meanMF_beamformGain = pow2db(mean(db2pow(MF_beamformGain)));

fprintf('Mean beamforming gain: %.1f dB\n', meanMF_beamformGain)

meanNonCoherentPower = pow2db(mean(db2pow(nonCoherenctPower)));

fprintf('Mean non-coherent beamformin gain: %.1f dB\n', meanNonCoherentPower)






