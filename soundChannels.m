% H_est = function soundChannels(...
% 	USE_WARPLAB_TXRX,
% 	 numRxAntennas, 
% 	 numTxAntennas,
% 	 snr_dB,

% 	 )



	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 1.2 Base station transmits its pilots
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	txLength = length(trainSignal);

	if USE_WARPLAB_TXRX == false;

		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Simulated channel
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% generate a random channel matrix
		H =  1./sqrt(2) * (randn(numRxAntennas,numTxAntennas) ...
	               + j*randn(numRxAntennas,numTxAntennas)); 

		% generate an AWGN noise vector
		rx_noise = (1/(sqrt(2)*db2mag(snr_dB))) *  complex(randn(numRxAntennas,txLength), ...
		                                         randn(numRxAntennas,txLength)) .';

		% apply the channel model. 
		rx_IQ = (H * trainSignal .') .' + rx_noise;
		rx_RSSI = mag2db(abs(rx_IQ));

	else
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Transmit and receive signal using WARPLab
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
	% 1.3 Signal processing to estimate channel from received signal
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
	if plotWaveforms

		figure('Name', 'Received Signal Sounding Phase');
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
	end % plot waveforms
% end
