
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% 1.2 Base station transmits its pilots
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	txLength = length(trainSignal);

	if USE_WARPLAB_TXRX == false;

		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Simulated channel
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% 
		if MODEL_NOISE == true
			% generate an AWGN noise vector
			rx_noise = (1/(sqrt(2)*db2mag(snr_dB))) *  complex(randn(numRxAntennas,txLength), ...
		                                         randn(numRxAntennas,txLength)) .';
			user_noise = (1/(sqrt(2)*db2mag(snr_dB))) *  complex(randn(numUsers,txLength), ...
		                                         randn(numUsers,txLength)) .';
		else
			rx_noise = zeros(numRxAntennas,txLength).';
			user_noise = zeros(numRxAntennas,txLength).';
		end

		% apply the channel model. 
		bs_IQ = (H_selfInt * trainSignal .') .' + rx_noise;
		bs_RSSI = mag2db(abs(bs_IQ));

		user_IQ = (H_user * trainSignal .') .' + user_noise;
		user_RSSI = mag2db(abs(user_IQ));
	else
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Transmit and receive signal using WARPLab
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		%Set up the baseband for the experiment
		wl_basebandCmd(nodes,'tx_delay',0);
		wl_basebandCmd(nodes,'tx_length',txLength);

		wl_basebandCmd(node_tx,[RFA, RFB], 'write_IQ', trainSignal);
		wl_interfaceCmd(node_tx, RFA+RFB,'tx_en');
		wl_interfaceCmd(node_rx, RFA+RFB,'rx_en');

		wl_basebandCmd(node_tx, RFA+RFB,'tx_buff_en');
		wl_basebandCmd(node_rx, RFA+RFB,'rx_buff_en');

		eth_trig.send();
		rx_IQ = wl_basebandCmd(node_rx,[RFA, RFB],'read_IQ', 0, txLength);

		bs_IQ = rx_IQ(:,1);
		user_IQ = rx_IQ(:,2);

		bs_RSSI = mag2db(abs(bs_IQ));  % for now. We'll fix this later.  
		user_RSSI = mag2db(abs(user_IQ));  % for now. We'll fix this later.   

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
	% rx_pilots = zeros(pilotLength, numTxAntennas*numRxAntennas);
	% for txAntenna = 1:numTxAntennas

	% 	pilots = bs_IQ(txPilotStartIndices(txAntenna)+1 ...
	% 		           : txPilotStartIndices(txAntenna)+pilotLength,:);

	% 	rx_pilots(:,(txAntenna-1)*numRxAntennas+1 : ...
	% 		         txAntenna*numRxAntennas) = pilots;
							    
	% end

	% usr_pilots = zeros(pilotLength, numTxAntennas*numUsers);
	% for txAntenna = 1:numTxAntennas

	% 	pilots = user_IQ(txPilotStartIndices(txAntenna)+1 ...
	% 		           : txPilotStartIndices(txAntenna)+pilotLength,:);

	% 	usr_pilots(:,(txAntenna-1)*numUsers+1 : ...
	% 		         txAntenna*numUsers) = pilots;
							    
	% end

	rx_pilots = zeros(pilotLength,0);
	user_pilots = zeros(pilotLength,0);
	for txAntenna = 1:numTxAntennas
		rx_pilots = [rx_pilots, bs_IQ(txPilotStartIndices(txAntenna)+1 : ...
			                          txPilotStartIndices(txAntenna)+pilotLength,:)];

		user_pilots = [user_pilots, user_IQ(txPilotStartIndices(txAntenna)+1 : ...
			                          txPilotStartIndices(txAntenna)+pilotLength,:)];
	end



	% just divide each by the pilots
	H_est_selfInt = reshape(mean(rx_pilots ./ repmat(signalPilot,1,numTxAntennas*numRxAntennas)),numRxAntennas,numTxAntennas);

	H_est_user = reshape(mean(user_pilots ./ repmat(signalPilot,1,numTxAntennas*numUsers)),numUsers,numTxAntennas);

	if USE_WARPLAB_TXRX == false && VERBOSE;

		% compute the average channel estimatio error
		selfIntChannelEstimationError = pow2db (mean(reshape(abs(H_selfInt - H_est_selfInt).^2 ./ abs(H_selfInt).^2, ...
			numTxAntennas*numRxAntennas,1) ));
		fprintf('SI Channel estimation error is %.2f dB\n', selfIntChannelEstimationError)


		% compute the average channel estimatio error
		userChannelEstimationError = pow2db (mean(reshape(abs(H_user - H_est_user).^2 ./ abs(H_user).^2, ...
			numTxAntennas*numUsers,1) ));
		fprintf('User Channel estimation error is %.2f dB\n', userChannelEstimationError)
	end

% end
