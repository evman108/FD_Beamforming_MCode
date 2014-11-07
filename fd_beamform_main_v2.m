%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% In this experiment, we will have an array of M_T transmit antennas.
% M_R receive antennas, K Users. The simplest, first-cut case will be
% Two transmit antennas (M_T = 2) One user, K=1, and one receive antenna,
% M_R = 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%
%Notes: because the beamforming is not applied to the preamble, we clip once we beamform. 
%Need a better way to do all this. Perhaps RSSI is better. 
%%%

clear all; 
clc; 
close all;
cf = 0;


%%%%%%%%%%%%%%%%%  User Parameters  %%%%%%%%%%%%%%
USE_WARPLAB_TXRX = true;

DEBUG_CANCELLATION = false;

PLOT_CHANNEL_SOUNDING = false;

PLOT_RX_DATA = true;

VERBOSE = false;

numPkts = 10;


%%%%%%%%% WARPLab Parameters %%%%%%%%%%%%%%%
%External trigger mode requires a connection from TRIGOUT_D0 on node 0
% to TRIGIN_D3 on node 1 (see http://warpproject.org/w/WARPLab/Examples for details)
USE_EXTERNAL_TRIGGER = true;
TXRX_DELAY_WARP = 0; % not exact
USE_AGC = false;
TXRX_DELAY_WARP = 45;

%Use sane defaults for hardware-dependent params in sim-only version
TXRX_DELAY_SIM = 0;
maxTxLength = 32768;
sampFreq = 40e6;
Ts = 1/sampFreq;


NUMNODES = 2; %must be set to 2 for now

% choose how many transmit and receive antennas
% you wish to use. Can be arbitrarty in simulation, 
% when using with WARP boards, must be less than 4. 
numTxAntennas = 4;
numRxAntennas = 1;
numUsers = 1; % must be one 

if numUsers > 1
	error('I cannot handle multiple users yet')
end

if numTxAntennas > 50
	warning('Pilots nearly consuming whole packet')
end

if USE_WARPLAB_TXRX
	TXRX_DELAY = TXRX_DELAY_WARP;
else
	TXRX_DELAY = TXRX_DELAY_SIM;
end

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
	
fd_beamform_warplabsetup_v2


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



zeroForce.selfIntSuppression = zeros(1,numPkts);
matchedFilter.selfIntSuppression = zeros(1,numPkts);
zeroForce.beamformGain = zeros(1,numPkts);
matchedFilter.beamformGain = zeros(1,numPkts);

plotWaveforms = true ;

for pktIndx = 1:numPkts

	if VERBOSE
		fprintf('\n\nPacket # %d:\n', pktIndx)
	else
		perccount(pktIndx,numPkts)
	end % VERBOSE

	if pktIndx > 1
		plotWaveforms = false ;
	end

	if USE_WARPLAB_TXRX == false

		% generate a random channel matrix that will hold for all packet transmisstion
		% in this loop
		if MODEL_FADING	== true

			H_selfInt = 1./sqrt(2) * (randn(numRxAntennas,numTxAntennas) ...
	                                  + j*randn(numRxAntennas,numTxAntennas)); 

			H_user = 1./sqrt(2) * (randn(numUsers,numTxAntennas) ...
                                   + j*randn(numUsers,numTxAntennas)); 
		else
			error('deterministic channels not implmented')
		end

	end % USE_WARPLAB_TXRX == false

	% This script transmit the training packet and has all the users estimate their channel
	% responses. % It is  global script. The only value that will be set (hopefully)
	% is H the simulated channel and H_est the estimated channel matrix. 
	fd_beamform_soundChannels_v2

	if plotWaveforms && PLOT_CHANNEL_SOUNDING
		plot_IQ(bs_IQ, bs_RSSI, numRxAntennas, 'BS Rx, Channel Sounding')
		plot_IQ(user_IQ, user_RSSI, numUsers, 'User Rx, Channel Sounding')
	end




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
	% 2.1 Create the transit signal, zero-force to 
	% to rx antennas, and user rest to beamform to 
	% intednded users. 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	payloadLength = maxTxLength - length(trainSignal);

	txLength = maxTxLength;

	% create time index for the payload
	t = [0:Ts:((payloadLength-1))*Ts].'; 

	payload = signalAmplitude * exp(t*j*2*pi*payloadToneFreq); %5 MHz sinusoid as our "payload" for RFA

	% null combining
	selfIntNullspace = null(H_est_selfInt);


	if isempty(selfIntNullspace)
		precoder = ones(numTxAntennas,1)./sqrt(numTxAntennas);
		warning('Cannot zero force')
	else
		precoder = selfIntNullspace(:,1);
	end

	precoderPower = pow2db(sum(sum(abs(precoder).^2)));


	if abs(precoderPower - 0) > 1e-6 
		error('Zero-forcing precoder does not have unity power')
	end

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
	fd_beamform_txrx_v2

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Visualize results for channel estimation 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	if plotWaveforms && PLOT_RX_DATA
		plot_IQ(bs_IQ, bs_RSSI, numRxAntennas, 'BS Rx, Zero-forcing')
		plot_IQ(user_IQ, user_RSSI, numUsers, 'User Rx, Zero-forcing')
	end % plot plotWaveforms

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.3 Users and base station process their received signals.
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Visualize results for channel estimation 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 2.3.2 Payload processing
	%%%%%%%%%%%%%%%%%%%%%%%%%%

	payloadStart = length(trainSignal) + TXRX_DELAY + 1;

	rx_payload = bs_IQ(payloadStart:end);

	user_payload = user_IQ(payloadStart:end);

	%rx_payload_eq = rx_payload ./ sum(H_eq);
	rx_payload_eq = rx_payload ;

	user_payload_eq = rx_payload ;

	% there will be some error at first and last, so igore that
	% n_samp_ignore = 0;
	% precodingError = pow2db(mean(abs(payload(n_samp_ignore+1:length(rx_payload_eq)-n_samp_ignore) - rx_payload_eq(n_samp_ignore+1:end-n_samp_ignore)).^2));


	rssiPreambleStop = length(preamble);

	rssiPreambleStart = rssiPreambleStop - numSampsForUncodedEstimation;

	bs_RSSI_Samples = bs_IQ(rssiPreambleStart + TXRX_DELAY : rssiPreambleStop + TXRX_DELAY);

	selfIntUnsupressed = pow2db(mean(abs(bs_RSSI_Samples).^2));

	selfIntSupressed = pow2db(mean(abs(rx_payload).^2));

	if USE_WARPLAB_TXRX == false
		noiseFloor = pow2db(mean(abs(rx_noise(payloadStart:end)).^2));
	end

	zeroForce.selfIntSuppression(pktIndx) = selfIntUnsupressed - selfIntSupressed;

	user_RSSI_Samples = user_IQ(rssiPreambleStart + TXRX_DELAY : rssiPreambleStop + TXRX_DELAY);

	userNoncoherentPower = pow2db(mean(abs(user_RSSI_Samples).^2));

	userCoherentPower = pow2db(mean(abs(user_payload).^2));

	zeroForce.beamformGain(pktIndx) = userCoherentPower - userNoncoherentPower;


	if plotWaveforms && DEBUG_CANCELLATION	
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
	fd_beamform_soundChannels_v2


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 3.2 Create the transit signal, matched filter
	% to users the ignores the self-interference
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	payloadLength = maxTxLength - length(trainSignal);

	txLength = maxTxLength;

	% create time index for the payload
	t = [0:Ts:((payloadLength-1))*Ts].'; 

	payload = signalAmplitude*exp(t*j*2*pi*payloadToneFreq); %5 MHz sinusoid as our "payload" for RFA

	% modulate with a a quare wave at mush lower frequency
	% payload = (.5 * (1 + square(t*payloadToneFreq/10))) .* payload; 

	% conjugate beamforming
	% precoder = ctranspose(H_est_selfInt)./ sqrt(sum(abs(H_est_selfInt).^2)) ;
	%precoder = ctranspose(H_est_user)./ sqrt(sum(abs(H_est_user).^2)) ;

	precoder = ctranspose(H_est_user) ./sqrt(sum(sum(abs(H_est_user).^2)));


	% Blind precoder 
	%precoder =[1; 1]./sqrt(numTxAntennas);

	precoderPower = pow2db(sum(sum(abs(precoder).^2)));


	if abs(precoderPower - 0) > 1e-6 
		error('Zero-forcing precoder does not have unity power')
	end


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

	fd_beamform_txrx_v2

	if plotWaveforms && PLOT_RX_DATA
		plot_IQ(bs_IQ, bs_RSSI, numRxAntennas, 'BS Rx, Matched Filter')
		plot_IQ(user_IQ, user_RSSI, numUsers, 'User Rx, Matched Filter')
	end 


	%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 3.3.2 Payload processing
	%%%%%%%%%%%%%%%%%%%%%%%%%%

	rx_payload = bs_IQ(payloadStart:end);

	user_payload = user_IQ(payloadStart:end);

	%rx_payload_eq = rx_payload ./ sum(H_eq);
	rx_payload_eq = rx_payload ;

	user_payload_eq = rx_payload ;

	% there will be some error at first and last, so igore that
	% n_samp_ignore = 0;
	% precodingError = pow2db(mean(abs(payload(n_samp_ignore+1:length(rx_payload_eq)-n_samp_ignore) - rx_payload_eq(n_samp_ignore+1:end-n_samp_ignore)).^2));


	rssiPreambleStop = length(preamble);

	rssiPreambleStart = rssiPreambleStop - numSampsForUncodedEstimation;

	bs_RSSI_Samples = bs_IQ(rssiPreambleStart + TXRX_DELAY : rssiPreambleStop + TXRX_DELAY);

	selfIntUnsupressed = pow2db(mean(abs(bs_RSSI_Samples).^2));

	selfIntSupressed = pow2db(mean(abs(rx_payload).^2));

	if USE_WARPLAB_TXRX == false
		noiseFloor = pow2db(mean(abs(rx_noise(payloadStart:end)).^2));
	end

	matchedFilter.selfIntSuppression(pktIndx) = selfIntUnsupressed - selfIntSupressed;

	user_RSSI_Samples = user_IQ(rssiPreambleStart + TXRX_DELAY : rssiPreambleStop + TXRX_DELAY);

	userNoncoherentPower = pow2db(mean(abs(user_RSSI_Samples).^2));

	userCoherentPower = pow2db(mean(abs(user_payload).^2));

	matchedFilter.beamformGain(pktIndx) = userCoherentPower - userNoncoherentPower;

	if plotWaveforms && DEBUG_CANCELLATION	
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

zeroForce.meanSuppression = pow2db(mean(db2pow(zeroForce.selfIntSuppression)));
fprintf('\nZero Force, mean self-interference suppression: %.1f dB\n',...
		 zeroForce.meanSuppression)

zeroForce.meanBeamformGain = pow2db(mean(db2pow(zeroForce.beamformGain)));
fprintf('Zero Force, mean beamforming gain: %.1f dB\n',...
		 zeroForce.meanBeamformGain )

matchedFilter.meanSuppression = pow2db(mean(db2pow(matchedFilter.selfIntSuppression)));
fprintf('\nMatched Filter, mean self-interference suppression: %.1f dB\n',...
		 matchedFilter.meanSuppression)

matchedFilter.meanBeamformGain = pow2db(mean(db2pow(matchedFilter.beamformGain)));
fprintf('Matched Filter, mean beamforming gain: %.1f dB\n',...
		 matchedFilter.meanBeamformGain )







